"""Read official STEP assembly with OCCT/XCAF; never modifies the source CAD."""

import argparse
import gzip
import hashlib
import json
import os
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--ocp-runtime", type=Path)
    parser.add_argument("--output", type=Path, default=Path("output/step_audit"))
    parser.add_argument("--export-geometry", type=Path)
    args = parser.parse_args()
    if args.ocp_runtime:
        sys.path.insert(0, str(args.ocp_runtime.resolve()))
        # 必须在 import OCP 之前把 OCP 的 DLL 目录挂上（Windows 专用）；
        # 句柄本身要留到进程结束才有效，故用下划线前缀保留而不被 ruff 判为未用变量。
        _dll_handle = (
            os.add_dll_directory(str(args.ocp_runtime.resolve()))
            if os.name == "nt"
            else None
        )
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GeomAbs import GeomAbs_Plane
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import (
        TopAbs_FACE,
        TopAbs_REVERSED,
        TopAbs_SHELL,
        TopAbs_VERTEX,
    )
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

    def bounds(shape):
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape, box, False, False)
        return list(box.Get())

    def name(label):
        attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
            return attr.Get().ToExtString()
        return ""

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(str(args.source.resolve())) != IFSelect_RetDone:
        raise RuntimeError("STEP import failed")
    doc = TDocStd_Document(TCollection_ExtendedString("RMUL"))
    if not reader.Transfer(doc):
        raise RuntimeError("XCAF document transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    records = []
    mesh_records = []

    def walk(label, parent_location, path):
        location = parent_location.Multiplied(shape_tool.GetLocation_s(label))
        definition = TDF_Label()
        is_reference = shape_tool.GetReferredShape_s(label, definition)
        if not is_reference:
            definition = label
        item_name = name(label) or name(definition)
        node_path = path + [item_name]
        children = TDF_LabelSequence()
        if shape_tool.GetComponents_s(definition, children):
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), location, node_path)
            return
        shape = shape_tool.GetShape_s(definition).Located(location)
        if args.export_geometry:
            mesher = BRepMesh_IncrementalMesh(shape, 0.1, False, 0.1, True)
            mesher.Perform()
            if not mesher.IsDone():
                raise RuntimeError("STEP tessellation failed")
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, props)
        center = props.CentreOfMass()
        colors = {}
        for kind in [
            XCAFDoc_ColorType.XCAFDoc_ColorGen,
            XCAFDoc_ColorType.XCAFDoc_ColorSurf,
            XCAFDoc_ColorType.XCAFDoc_ColorCurv,
        ]:
            color = Quantity_Color()
            for origin in (label, definition):
                if color_tool.GetColor_s(origin, kind, color):
                    colors[str(kind)] = [color.Red(), color.Green(), color.Blue()]
                    break
        vertices = set()
        explorer = TopExp_Explorer(shape, TopAbs_VERTEX)
        while explorer.More():
            v = BRep_Tool.Pnt_s(TopoDS.Vertex_s(explorer.Current()))
            vertices.add(tuple(round(c, 7) for c in (v.X(), v.Y(), v.Z())))
            explorer.Next()
        bbox = bounds(shape)
        faces = []
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            face = TopoDS.Face_s(exp.Current())
            fcolors = {}
            srgb = None
            for kind in [
                XCAFDoc_ColorType.XCAFDoc_ColorGen,
                XCAFDoc_ColorType.XCAFDoc_ColorSurf,
            ]:
                color = Quantity_Color()
                if color_tool.GetColor(face, kind, color):
                    fcolors[str(kind)] = [color.Red(), color.Green(), color.Blue()]
                    srgb = list(color.Values(Quantity_TOC_sRGB))
            if srgb is None:
                for kind in [
                    XCAFDoc_ColorType.XCAFDoc_ColorSurf,
                    XCAFDoc_ColorType.XCAFDoc_ColorGen,
                ]:
                    inherited = Quantity_Color()
                    if color_tool.GetColor_s(definition, kind, inherited):
                        srgb = list(inherited.Values(Quantity_TOC_sRGB))
                        break
            fsurf = BRepAdaptor_Surface(face)
            normal = None
            if fsurf.GetType() == GeomAbs_Plane:
                n = fsurf.Plane().Axis().Direction()
                s = -1 if face.Orientation() == TopAbs_REVERSED else 1
                normal = [s * n.X(), s * n.Y(), s * n.Z()]
            fp = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, fp)
            faces.append(
                {
                    "index": len(faces),
                    "bounds_mm": bounds(face),
                    "normal": normal,
                    "area_mm2": fp.Mass(),
                    "colors": fcolors,
                }
            )
            if args.export_geometry:
                loc = TopLoc_Location()
                mesh = BRep_Tool.Triangulation_s(face, loc)
                if mesh is None:
                    raise RuntimeError(
                        f"Face missing mesh: {len(records)}/{len(faces)-1}"
                    )
                triangles = []
                for t in range(1, mesh.NbTriangles() + 1):
                    indices = list(mesh.Triangle(t).Get())
                    if face.Orientation() == TopAbs_REVERSED:
                        indices[1], indices[2] = indices[2], indices[1]
                    # The standalone perimeter-cap open shell is exported inside-out.
                    # Reverse its winding, preserving all positions and analytic faces.
                    if len(records) == 1:
                        indices[1], indices[2] = indices[2], indices[1]
                    tri = []
                    for j in indices:
                        p = mesh.Node(j).Transformed(loc.Transformation())
                        tri.append(
                            [
                                round((6000 - p.X()) / 1000, 10),
                                round((4000 - p.Z()) / 1000, 10),
                                round(-p.Y() / 1000, 10),
                            ]
                        )
                    triangles.append(tri)
                mesh_records.append(
                    {
                        "occurrence": len(records),
                        "face": len(faces) - 1,
                        "srgb": srgb or [0.5, 0.5, 0.5],
                        "triangles_m": triangles,
                    }
                )
            exp.Next()
        shells = []
        exp = TopExp_Explorer(shape, TopAbs_SHELL)
        while exp.More():
            shells.append(bounds(exp.Current()))
            exp.Next()
        records.append(
            {
                "index": len(records),
                "name": item_name,
                "path": node_path,
                "bounds_mm": bbox,
                "size_mm": [bbox[i + 3] - bbox[i] for i in range(3)],
                "volume_mm3": props.Mass(),
                "center_mm": [center.X(), center.Y(), center.Z()],
                "colors": colors,
                "vertices_mm": sorted(vertices),
                "faces": faces,
                "shells": shells,
            }
        )

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    args.output.mkdir(parents=True, exist_ok=True)
    result = {
        "source": args.source.name,
        "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
        "reader": "OCCT 7.7.2 STEPCAFControl (AP214 assembly)",
        "length_units": "millimetres",
        "roots": roots.Length(),
        "occurrences": len(records),
        "parts": records,
    }
    (args.output / "assembly.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if args.export_geometry:
        geometry = {
            "source": result["source"],
            "source_sha256": result["source_sha256"],
            "units": "metres",
            "linear_deflection_mm": 0.1,
            "angular_deflection_rad": 0.1,
            "transform": "sim x=(6000-X)/1000, sim y=(4000-Z)/1000, sim z=-Y/1000; CAD mm, -Y up; red NW by AP214 red marker",
            "perimeter_cap_winding_reversed": True,
            "faces": mesh_records,
        }
        args.export_geometry.parent.mkdir(parents=True, exist_ok=True)
        args.export_geometry.write_bytes(
            gzip.compress(json.dumps(geometry, separators=(",", ":")).encode(), mtime=0)
        )
        print("EXPORTED TRIANGLES", sum(len(f["triangles_m"]) for f in mesh_records))
    for r in records:
        print(
            json.dumps(
                {k: r[k] for k in ["index", "name", "bounds_mm", "size_mm", "colors"]},
                ensure_ascii=False,
            )
        )
    print("OCCURRENCES", len(records))


if __name__ == "__main__":
    main()
