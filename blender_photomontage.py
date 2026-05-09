#!/usr/bin/env python3
"""Render a camera-calibrated Blender photomontage from geometry/*.json.

Run with Blender:

    blender --background --python blender_photomontage.py -- \
      --geometry terassi_puu \
      --calibration kuvat/IMG_2837_viewer_camera.json \
      --output kuvat/IMG_2837_blender.png \
      --scale 0.5 \
      --samples 96 \
      --sun-azimuth-deg 291.0 \
      --sun-elevation-deg 8.0 \
      --ai-guides

The camera calibration JSON is exported from viewer.py. Geometry coordinates are
converted with the same mapping as the viewer: {x,y,z} -> Blender/Three {x,z,y}.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
FALLBACK_SUN_AZIMUTH_DEG = 290.9966
FALLBACK_SUN_ELEVATION_DEG = 7.9930
DEFAULT_COLUMN_COLOR = (0.78, 0.67, 0.48, 1.0)


def parse_args() -> argparse.Namespace:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry", default="terassi_puu", help="Geometry name without .json")
    parser.add_argument("--calibration", default="kuvat/IMG_2837_viewer_camera.json")
    parser.add_argument("--output", default="kuvat/IMG_2837_blender_photomontage.png")
    parser.add_argument("--scale", type=float, default=0.5, help="Render scale relative to source photo")
    parser.add_argument("--samples", type=int, default=96, help="Cycles sample count")
    parser.add_argument("--engine", choices=["CYCLES", "EEVEE"], default="CYCLES")
    parser.add_argument("--save-blend", default="", help="Optional .blend output path")
    parser.add_argument(
        "--sun-azimuth-deg",
        type=float,
        default=None,
        help="Manual true sun azimuth in degrees clockwise from north. Defaults to EXIF GPS/time when available.",
    )
    parser.add_argument(
        "--sun-elevation-deg",
        type=float,
        default=None,
        help="Manual sun elevation above horizon in degrees. Defaults to EXIF GPS/time when available.",
    )
    parser.add_argument(
        "--sun-source",
        choices=["auto", "exif", "manual"],
        default="auto",
        help="How to resolve sun azimuth/elevation: auto uses EXIF GPS/time when possible, exif requires it, manual uses --sun-*-deg or fallback defaults.",
    )
    parser.add_argument(
        "--sun-timezone-offset-hours",
        type=float,
        default=None,
        help="Timezone offset for EXIF DateTime values that lack OffsetTime, e.g. 3 for Finnish summer time. Defaults to the local system timezone.",
    )
    parser.add_argument(
        "--camera-heading-deg",
        type=float,
        default=None,
        help="Camera true heading in degrees clockwise from north. Defaults to EXIF GPSImgDirection from the calibration image.",
    )
    parser.add_argument(
        "--sun-facing-camera",
        action="store_true",
        help="Place the sun on the calibrated camera view direction instead of using true azimuth/heading mapping.",
    )
    parser.add_argument("--sun-energy", type=float, default=2.8, help="Blender Sun light energy.")
    parser.add_argument("--sun-angle-deg", type=float, default=1.4, help="Sun angular size / shadow softness.")
    parser.add_argument(
        "--sun-color",
        default="auto",
        help="Sun light color as #RRGGBB or r,g,b. 'auto' warms low sun and uses near-neutral daylight for high sun.",
    )
    parser.add_argument("--sky-fill-energy", type=float, default=260.0, help="Soft sky fill area light energy.")
    parser.add_argument(
        "--no-bounce-surfaces",
        dest="bounce_surfaces",
        action="store_false",
        help="Disable camera-invisible reference/floor bounce surfaces used for indirect light.",
    )
    parser.set_defaults(bounce_surfaces=True)
    parser.add_argument("--bounce-wall-reflectance", type=float, default=0.78, help="Diffuse reflectance for wall bounce surfaces.")
    parser.add_argument("--bounce-floor-reflectance", type=float, default=0.62, help="Diffuse reflectance for terrace/floor bounce surfaces.")
    parser.add_argument("--bounce-wall-energy", type=float, default=0.42, help="Emission strength for camera-invisible wall bounce light.")
    parser.add_argument("--bounce-floor-energy", type=float, default=0.34, help="Emission strength for camera-invisible floor bounce light.")
    parser.add_argument("--bounce-floor-z-mm", type=float, default=0.0, help="Z level for the automatic terrace floor bounce plane.")
    parser.add_argument("--bounce-floor-padding-mm", type=float, default=600.0, help="Padding around render geometry for automatic floor bounce plane.")
    parser.add_argument(
        "--no-solar-sky-reflection",
        dest="solar_sky_reflection",
        action="store_false",
        help="Disable camera-invisible sky reflection cards used to give solar panel glass a sky reflection.",
    )
    parser.set_defaults(solar_sky_reflection=True)
    parser.add_argument("--solar-sky-reflection-energy", type=float, default=0.55, help="Emission strength for the solar panel sky reflection card.")
    parser.add_argument("--solar-sky-reflection-height-mm", type=float, default=6500.0, help="Height above the new structure for the solar panel sky reflection card.")
    parser.add_argument("--solar-sky-reflection-padding-mm", type=float, default=12000.0, help="XY padding around render geometry for the solar panel sky reflection card.")
    parser.add_argument(
        "--column-color",
        default="",
        help="Manual column material color as #RRGGBB or r,g,b values. Overrides automatic photo color matching.",
    )
    parser.add_argument(
        "--no-column-color-match",
        dest="column_color_match",
        action="store_false",
        help="Disable automatic column color sampling from existing columns in the source photo.",
    )
    parser.set_defaults(column_color_match=True)
    parser.add_argument("--column-color-sample-geometry", default="katos", help="Geometry used for existing column color sampling.")
    parser.add_argument(
        "--column-color-sample-ids",
        default="col.x125.outer.bottom,col.x3600.outer.bottom,col.x7075.outer.bottom,col.x125,col.x7075",
        help="Comma-separated existing column ids sampled from the source photo for automatic column color matching.",
    )
    parser.add_argument("--column-color-sample-erode-px", type=int, default=3, help="Erode the column sample mask by this many render pixels before sampling.")
    parser.add_argument("--column-color-match-min-pixels", type=int, default=80, help="Minimum masked pixels required for automatic column color matching.")
    parser.add_argument("--column-color-match-strength", type=float, default=0.85, help="Blend strength from default column color to sampled/manual column color.")
    parser.add_argument("--ai-guides", action="store_true", help="Also render structure, mask, lineart and depth guide images.")
    parser.add_argument("--guide-samples", type=int, default=8, help="Sample count for AI guide renders.")
    parser.add_argument("--mask-grow-px", type=int, default=10, help="Dilate the inpaint mask by this many output pixels.")
    parser.add_argument("--mask-blur-px", type=float, default=3.0, help="Blur radius for the soft inpaint mask.")
    parser.add_argument("--line-threshold", type=int, default=130, help="Luma threshold for binary lineart post-processing.")
    parser.add_argument(
        "--no-foreground-occlusion",
        dest="foreground_occlusion",
        action="store_false",
        help="Disable restoring original photo pixels over existing foreground terrace structures.",
    )
    parser.set_defaults(foreground_occlusion=True)
    parser.add_argument("--foreground-occlusion-geometry", default="katos", help="Geometry file used for foreground occluders.")
    parser.add_argument(
        "--foreground-occluder-ids",
        default="beam.bottom.conreate.0,beam.bottom.conreate.1,col.x125.outer.bottom,col.x3600.outer.bottom,col.x7075.outer.bottom",
        help="Comma-separated existing member ids rendered into the foreground occlusion mask.",
    )
    parser.add_argument(
        "--foreground-extra-occlusion-geometries",
        default="portaikko",
        help="Comma-separated extra geometry names whose members and surfaces are rendered into the foreground occlusion mask.",
    )
    parser.add_argument(
        "--foreground-extra-member-margin-mm",
        type=float,
        default=0.0,
        help="Grow all member boxes from extra foreground occlusion geometries by this many millimetres.",
    )
    parser.add_argument(
        "--foreground-extra-rafter-margin-mm",
        type=float,
        default=0.0,
        help="Additional margin for rafters and purlins from extra foreground occlusion geometries.",
    )
    parser.add_argument("--foreground-extra-mask-grow-left-px", type=int, default=0, help="Grow extra foreground geometry mask left in output pixels.")
    parser.add_argument("--foreground-extra-mask-grow-right-px", type=int, default=0, help="Grow extra foreground geometry mask right in output pixels.")
    parser.add_argument("--foreground-extra-mask-grow-up-px", type=int, default=0, help="Grow extra foreground geometry mask up in output pixels.")
    parser.add_argument("--foreground-extra-mask-grow-down-px", type=int, default=0, help="Grow extra foreground geometry mask down in output pixels.")
    parser.add_argument("--foreground-extra-mask-blur-px", type=float, default=0.0, help="Blur extra foreground mask before merging into the main foreground mask.")
    parser.add_argument("--foreground-edge-top-mm", type=float, default=0.0, help="Top z of the extra terrace-front occlusion band.")
    parser.add_argument("--foreground-edge-bottom-mm", type=float, default=-650.0, help="Bottom z of the extra terrace-front occlusion band.")
    parser.add_argument("--foreground-mask-grow-px", type=int, default=2, help="Dilate foreground occlusion mask by this many pixels.")
    parser.add_argument("--foreground-mask-blur-px", type=float, default=0.8, help="Blur radius for foreground occlusion mask edges.")
    parser.add_argument(
        "--no-foregroud-occlusion",
        dest="foreground_occlusion",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    return parser.parse_args(argv)


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def exif_tag_map(image_path: Path) -> tuple[dict, dict]:
    from PIL import ExifTags, Image

    image = Image.open(image_path)
    exif = image.getexif()
    tags = {ExifTags.TAGS.get(tag_id, tag_id): value for tag_id, value in exif.items()}
    try:
        gps = exif.get_ifd(ExifTags.IFD.GPSInfo)
    except Exception:
        gps = {}
    gps_tags = {ExifTags.GPSTAGS.get(tag_id, tag_id): value for tag_id, value in gps.items()}
    return tags, gps_tags


def rational_to_float(value) -> float:
    if isinstance(value, tuple) and len(value) == 2 and all(isinstance(v, int) for v in value):
        return value[0] / value[1]
    return float(value)


def gps_coord_to_decimal(value, ref: str | None) -> float | None:
    if not value or len(value) != 3:
        return None
    deg = rational_to_float(value[0])
    minutes = rational_to_float(value[1])
    seconds = rational_to_float(value[2])
    decimal = deg + minutes / 60 + seconds / 3600
    if ref in {"S", "W"}:
        decimal *= -1
    return decimal


def parse_exif_datetime(tags: dict, timezone_offset_hours: float | None) -> dt.datetime | None:
    text = tags.get("DateTimeOriginal") or tags.get("DateTimeDigitized") or tags.get("DateTime")
    if not text:
        return None
    if isinstance(text, bytes):
        text = text.decode("utf-8", "ignore")
    text = str(text).strip("\x00 ")
    try:
        value = dt.datetime.strptime(text, "%Y:%m:%d %H:%M:%S")
    except ValueError:
        return None

    offset_text = tags.get("OffsetTimeOriginal") or tags.get("OffsetTimeDigitized") or tags.get("OffsetTime")
    if isinstance(offset_text, bytes):
        offset_text = offset_text.decode("utf-8", "ignore")
    offset_text = str(offset_text).strip("\x00 ") if offset_text else ""
    match = re.fullmatch(r"([+-])(\d{2}):?(\d{2})", offset_text)
    if match:
        sign = 1 if match.group(1) == "+" else -1
        hours = int(match.group(2))
        minutes = int(match.group(3))
        tz = dt.timezone(sign * dt.timedelta(hours=hours, minutes=minutes))
        return value.replace(tzinfo=tz)

    if timezone_offset_hours is not None:
        return value.replace(tzinfo=dt.timezone(dt.timedelta(hours=float(timezone_offset_hours))))

    local_tz = dt.datetime.now().astimezone().tzinfo
    return value.replace(tzinfo=local_tz)


def exif_datetime_location(
    image_path: Path,
    timezone_offset_hours: float | None,
) -> tuple[dt.datetime, float, float] | None:
    tags, gps = exif_tag_map(image_path)
    captured_at = parse_exif_datetime(tags, timezone_offset_hours)
    lat = gps_coord_to_decimal(gps.get("GPSLatitude"), gps.get("GPSLatitudeRef"))
    lon = gps_coord_to_decimal(gps.get("GPSLongitude"), gps.get("GPSLongitudeRef"))
    if captured_at is None or lat is None or lon is None:
        return None
    return captured_at, lat, lon


def solar_position_deg(captured_at: dt.datetime, latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
    if captured_at.tzinfo is None:
        raise ValueError("captured_at must be timezone-aware")
    utc = captured_at.astimezone(dt.timezone.utc)
    day = utc.timetuple().tm_yday
    hour = utc.hour + utc.minute / 60 + utc.second / 3600 + utc.microsecond / 3_600_000_000
    gamma = 2 * math.pi / 365 * (day - 1 + (hour - 12) / 24)
    equation_of_time_min = 229.18 * (
        0.000075
        + 0.001868 * math.cos(gamma)
        - 0.032077 * math.sin(gamma)
        - 0.014615 * math.cos(2 * gamma)
        - 0.040849 * math.sin(2 * gamma)
    )
    declination = (
        0.006918
        - 0.399912 * math.cos(gamma)
        + 0.070257 * math.sin(gamma)
        - 0.006758 * math.cos(2 * gamma)
        + 0.000907 * math.sin(2 * gamma)
        - 0.002697 * math.cos(3 * gamma)
        + 0.00148 * math.sin(3 * gamma)
    )
    utc_minutes = utc.hour * 60 + utc.minute + utc.second / 60 + utc.microsecond / 60_000_000
    true_solar_time = (utc_minutes + equation_of_time_min + 4 * longitude_deg) % 1440
    hour_angle_deg = true_solar_time / 4 - 180
    if hour_angle_deg < -180:
        hour_angle_deg += 360

    latitude = math.radians(latitude_deg)
    hour_angle = math.radians(hour_angle_deg)
    cos_zenith = (
        math.sin(latitude) * math.sin(declination)
        + math.cos(latitude) * math.cos(declination) * math.cos(hour_angle)
    )
    cos_zenith = max(-1.0, min(1.0, cos_zenith))
    zenith = math.acos(cos_zenith)
    elevation = 90 - math.degrees(zenith)
    azimuth = (
        math.degrees(
            math.atan2(
                math.sin(hour_angle),
                math.cos(hour_angle) * math.sin(latitude) - math.tan(declination) * math.cos(latitude),
            )
        )
        + 180
    ) % 360
    return azimuth, elevation


def resolve_sun_position(args: argparse.Namespace, calibration: dict) -> None:
    has_manual_azimuth = args.sun_azimuth_deg is not None
    has_manual_elevation = args.sun_elevation_deg is not None
    if has_manual_azimuth != has_manual_elevation:
        raise ValueError("Pass both --sun-azimuth-deg and --sun-elevation-deg, or neither.")
    if has_manual_azimuth or args.sun_source == "manual":
        args.sun_azimuth_deg = (
            float(args.sun_azimuth_deg) if args.sun_azimuth_deg is not None else FALLBACK_SUN_AZIMUTH_DEG
        )
        args.sun_elevation_deg = (
            float(args.sun_elevation_deg) if args.sun_elevation_deg is not None else FALLBACK_SUN_ELEVATION_DEG
        )
        args.sun_position_source = "manual"
        return

    image_path = ROOT / calibration["image"]["path"]
    exif_position = exif_datetime_location(image_path, args.sun_timezone_offset_hours)
    if exif_position:
        captured_at, lat, lon = exif_position
        args.sun_azimuth_deg, args.sun_elevation_deg = solar_position_deg(captured_at, lat, lon)
        args.sun_position_source = f"exif:{captured_at.isoformat()} lat={lat:.6f} lon={lon:.6f}"
        return

    if args.sun_source == "exif":
        raise RuntimeError(
            f"Could not resolve sun from EXIF GPS/time in {image_path}; pass manual --sun-azimuth-deg/--sun-elevation-deg."
        )
    args.sun_azimuth_deg = FALLBACK_SUN_AZIMUTH_DEG
    args.sun_elevation_deg = FALLBACK_SUN_ELEVATION_DEG
    args.sun_position_source = "fallback"


def parse_rgb_color(value: str) -> tuple[float, float, float, float] | None:
    text = value.strip()
    if not text:
        return None
    if text.startswith("#"):
        hex_text = text[1:]
        if len(hex_text) == 3:
            hex_text = "".join(ch * 2 for ch in hex_text)
        if len(hex_text) != 6:
            raise ValueError(f"Invalid color {value!r}; use #RRGGBB or r,g,b.")
        return tuple(int(hex_text[i : i + 2], 16) / 255 for i in (0, 2, 4)) + (1.0,)
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"Invalid color {value!r}; use #RRGGBB or r,g,b.")
    numbers = [float(part) for part in parts]
    if max(numbers) > 1:
        numbers = [value / 255 for value in numbers]
    return (
        max(0.0, min(1.0, numbers[0])),
        max(0.0, min(1.0, numbers[1])),
        max(0.0, min(1.0, numbers[2])),
        1.0,
    )


def blend_rgb(
    base: tuple[float, float, float, float],
    target: tuple[float, float, float, float],
    strength: float,
) -> tuple[float, float, float, float]:
    t = max(0.0, min(1.0, float(strength)))
    return (
        base[0] * (1 - t) + target[0] * t,
        base[1] * (1 - t) + target[1] * t,
        base[2] * (1 - t) + target[2] * t,
        1.0,
    )


def rgb_to_hex(color: tuple[float, float, float, float]) -> str:
    return "#{:02X}{:02X}{:02X}".format(
        round(max(0.0, min(1.0, color[0])) * 255),
        round(max(0.0, min(1.0, color[1])) * 255),
        round(max(0.0, min(1.0, color[2])) * 255),
    )


def auto_sun_color_for_elevation(elevation_deg: float) -> tuple[float, float, float, float]:
    low_sun = (1.0, 0.78, 0.52, 1.0)
    high_sun = (1.0, 0.98, 0.94, 1.0)
    t = (float(elevation_deg) - 8.0) / (35.0 - 8.0)
    return blend_rgb(low_sun, high_sun, t)


def resolve_sun_color(args: argparse.Namespace) -> None:
    value = str(args.sun_color).strip()
    if not value or value.lower() == "auto":
        args.sun_color_rgb = auto_sun_color_for_elevation(args.sun_elevation_deg)
        args.sun_color_source = "auto"
        return
    args.sun_color_rgb = parse_rgb_color(value)
    if args.sun_color_rgb is None:
        raise ValueError(f"Invalid sun color {value!r}")
    args.sun_color_source = "manual"


def exif_heading_deg(image_path: Path) -> float | None:
    _, gps = exif_tag_map(image_path)
    for name in ("GPSImgDirection", "GPSDestBearing"):
        if name in gps:
            return float(gps[name])
    return None


def resolve_camera_heading_deg(calibration: dict, override_deg: float | None) -> float:
    if override_deg is not None:
        return override_deg
    image_path = ROOT / calibration["image"]["path"]
    heading = exif_heading_deg(image_path)
    if heading is None:
        raise RuntimeError(
            f"Camera heading missing from {image_path}; pass --camera-heading-deg explicitly."
        )
    return heading


def geometry_true_north_xy(geo: dict) -> tuple[float, float] | None:
    true_north = (
        geo.get("project", {})
        .get("coordinate_system", {})
        .get("true_north")
    )
    if not true_north:
        return None
    x = float(true_north["x"])
    y = float(true_north["y"])
    length = math.hypot(x, y)
    if length <= 1e-9:
        raise ValueError("project.coordinate_system.true_north vector length must be non-zero")
    return x / length, y / length


def main() -> None:
    args = parse_args()
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))

    import bpy
    from mathutils import Matrix, Vector
    import geometry_loader

    def clear_scene() -> None:
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete()
        for block in (
            bpy.data.meshes,
            bpy.data.materials,
            bpy.data.images,
            bpy.data.lights,
            bpy.data.cameras,
        ):
            for item in list(block):
                if item.users == 0:
                    block.remove(item)

    def pt(p: dict | tuple[float, float, float]) -> Vector:
        if isinstance(p, dict):
            return Vector((float(p["x"]), float(p["z"]), float(p["y"])))
        x, y, z = p
        return Vector((float(x), float(z), float(y)))

    def three_matrix(elements: list[float]) -> Matrix:
        e = elements
        return Matrix((
            (e[0], e[4], e[8], e[12]),
            (e[1], e[5], e[9], e[13]),
            (e[2], e[6], e[10], e[14]),
            (e[3], e[7], e[11], e[15]),
        ))

    def material_principled(
        name: str,
        color: tuple[float, float, float, float],
        roughness: float = 0.55,
        metallic: float = 0.0,
        alpha: float = 1.0,
        transmission: float = 0.0,
        specular: float | None = None,
        coat_weight: float = 0.0,
        coat_roughness: float = 0.08,
        ior: float | None = None,
    ):
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
        mat.diffuse_color = color
        if alpha < 1.0:
            mat.blend_method = "BLEND"
            if hasattr(mat, "use_screen_refraction"):
                mat.use_screen_refraction = True
            mat.show_transparent_back = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            if "Base Color" in bsdf.inputs:
                bsdf.inputs["Base Color"].default_value = color
            if "Roughness" in bsdf.inputs:
                bsdf.inputs["Roughness"].default_value = roughness
            if "Metallic" in bsdf.inputs:
                bsdf.inputs["Metallic"].default_value = metallic
            if "Alpha" in bsdf.inputs:
                bsdf.inputs["Alpha"].default_value = alpha
            if "Transmission Weight" in bsdf.inputs:
                bsdf.inputs["Transmission Weight"].default_value = transmission
            if specular is not None:
                for input_name in ("Specular IOR Level", "Specular"):
                    if input_name in bsdf.inputs:
                        bsdf.inputs[input_name].default_value = specular
                        break
            if "Coat Weight" in bsdf.inputs:
                bsdf.inputs["Coat Weight"].default_value = coat_weight
            if "Coat Roughness" in bsdf.inputs:
                bsdf.inputs["Coat Roughness"].default_value = coat_roughness
            if ior is not None and "IOR" in bsdf.inputs:
                bsdf.inputs["IOR"].default_value = ior
        return mat

    def material_principled_with_emission(
        name: str,
        color: tuple[float, float, float, float],
        emission_color: tuple[float, float, float, float],
        emission_strength: float,
        roughness: float = 0.55,
        specular: float | None = None,
    ):
        mat = material_principled(name, color, roughness=roughness, specular=specular)
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            for input_name in ("Emission Color", "Emission"):
                if input_name in bsdf.inputs:
                    bsdf.inputs[input_name].default_value = emission_color
                    break
            for input_name in ("Emission Strength", "Emission Strength"):
                if input_name in bsdf.inputs:
                    bsdf.inputs[input_name].default_value = emission_strength
                    break
        return mat

    def material_emission_image(name: str, image_path: Path):
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        nodes.clear()
        out = nodes.new("ShaderNodeOutputMaterial")
        emission = nodes.new("ShaderNodeEmission")
        tex = nodes.new("ShaderNodeTexImage")
        tex.image = bpy.data.images.load(str(image_path))
        emission.inputs["Strength"].default_value = 1.0
        mat.node_tree.links.new(tex.outputs["Color"], emission.inputs["Color"])
        mat.node_tree.links.new(emission.outputs["Emission"], out.inputs["Surface"])
        return mat

    def material_emission_color(name: str, color: tuple[float, float, float, float], strength: float = 1.0):
        mat = bpy.data.materials.new(name)
        mat.use_nodes = True
        mat.diffuse_color = color
        nodes = mat.node_tree.nodes
        nodes.clear()
        out = nodes.new("ShaderNodeOutputMaterial")
        emission = nodes.new("ShaderNodeEmission")
        emission.inputs["Color"].default_value = color
        emission.inputs["Strength"].default_value = strength
        mat.node_tree.links.new(emission.outputs["Emission"], out.inputs["Surface"])
        return mat

    def create_mesh_object(name: str, verts: list[Vector], faces: list[list[int]], mat=None):
        mesh = bpy.data.meshes.new(name + "Mesh")
        mesh.from_pydata([tuple(v) for v in verts], [], faces)
        mesh.update(calc_edges=True)
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(obj)
        if mat:
            obj.data.materials.append(mat)
        return obj

    def rot_axis(v: Vector, axis: Vector, angle_rad: float) -> Vector:
        return v.copy().rotate(Matrix.Rotation(angle_rad, 4, axis)) or v

    def member_frame(start: Vector, end: Vector, section_rotation_deg: float = 0.0):
        direction = end - start
        length = direction.length
        if length < 1e-9:
            return None
        direction.normalize()
        world_up = Vector((0, 1, 0))
        ref_up = Vector((1, 0, 0)) if abs(direction.dot(world_up)) > 0.95 else world_up
        z_ax = direction.cross(ref_up)
        if z_ax.length < 1e-9:
            return None
        z_ax.normalize()
        y_ax = z_ax.cross(direction)
        y_ax.normalize()
        if abs(section_rotation_deg) > 1e-9:
            q = Matrix.Rotation(math.radians(section_rotation_deg), 4, direction)
            y_ax = q @ y_ax
            z_ax = q @ z_ax
            y_ax.normalize()
            z_ax.normalize()
        return {"dir": direction, "y": y_ax, "z": z_ax, "length": length}

    def member_ends(group: str, member: dict) -> tuple[Vector, Vector]:
        if group == "columns":
            return pt(member["base"]), pt(member["top"])
        return pt(member["axis_start"]), pt(member["axis_end"])

    def member_dims(member: dict) -> tuple[float, float]:
        profile = member.get("profile") or {}
        h_mm = float(profile.get("h_mm") or 0)
        b_mm = float(profile.get("b_mm") or 0) * float(profile.get("count", 1))
        return h_mm, b_mm

    def create_box_member(name: str, group: str, member: dict, mat, margin_mm: float = 0.0):
        start, end = member_ends(group, member)
        h_mm, b_mm = member_dims(member)
        frame = member_frame(start, end, float(member.get("section_rotation_deg", 0)))
        if frame is None or not h_mm or not b_mm:
            return None
        center = (start + end) * 0.5
        margin = max(0.0, float(margin_mm))
        length_half = frame["length"] / 2 + margin
        height_half = h_mm / 2 + margin
        width_half = b_mm / 2 + margin
        corners: dict[tuple[int, int, int], int] = {}
        verts: list[Vector] = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    p = (
                        center
                        + frame["dir"] * sx * length_half
                        + frame["y"] * sy * height_half
                        + frame["z"] * sz * width_half
                    )
                    corners[(sx, sy, sz)] = len(verts)
                    verts.append(p)
        faces = [
            [corners[(-1, -1, -1)], corners[(-1, 1, -1)], corners[(-1, 1, 1)], corners[(-1, -1, 1)]],
            [corners[(1, -1, -1)], corners[(1, -1, 1)], corners[(1, 1, 1)], corners[(1, 1, -1)]],
            [corners[(-1, -1, -1)], corners[(1, -1, -1)], corners[(1, -1, 1)], corners[(-1, -1, 1)]],
            [corners[(-1, 1, -1)], corners[(-1, 1, 1)], corners[(1, 1, 1)], corners[(1, 1, -1)]],
            [corners[(-1, -1, -1)], corners[(-1, 1, -1)], corners[(1, 1, -1)], corners[(1, -1, -1)]],
            [corners[(-1, -1, 1)], corners[(1, -1, 1)], corners[(1, 1, 1)], corners[(-1, 1, 1)]],
        ]
        obj = create_mesh_object(name, verts, faces, mat)
        return obj

    def build_member_index(geo: dict):
        idx = {}
        for group in ("columns", "beams", "rafters", "purlins"):
            for member in geometry_loader.expanded_members(geo, group):
                start, end = member_ends(group, member)
                idx[member["id"]] = {
                    "group": group,
                    "member": member,
                    "start": start,
                    "end": end,
                    "frame": member_frame(start, end, float(member.get("section_rotation_deg", 0))),
                    "object": None,
                }
        return idx

    def project_point_to_line(point: Vector, start: Vector, end: Vector) -> Vector:
        vec = end - start
        if vec.length < 1e-9:
            return start.copy()
        direction = vec.normalized()
        return start + direction * (point - start).dot(direction)

    def projected_member_half_extent(info, axis_dir: Vector) -> float:
        if not info or not info["frame"]:
            return 0.0
        h_mm, b_mm = member_dims(info["member"])
        frame = info["frame"]
        direction = axis_dir.normalized()
        return (
            abs(direction.dot(frame["dir"])) * frame["length"] / 2
            + abs(direction.dot(frame["y"])) * h_mm / 2
            + abs(direction.dot(frame["z"])) * b_mm / 2
        )

    def cut_member_end(cut: dict, con: dict, info) -> str:
        ref = cut.get("reference")
        if ref in ("axis_start", "axis_end"):
            return ref
        axis_point = project_point_to_line(pt(con["at"]), info["start"], info["end"])
        return "axis_start" if (axis_point - info["start"]).length_squared <= (axis_point - info["end"]).length_squared else "axis_end"

    def cut_local_frame(info, member_end: str):
        frame = info["frame"]
        if not frame:
            return None
        x_ax = -frame["dir"] if member_end == "axis_end" else frame["dir"].copy()
        y_ax = frame["y"].copy()
        z_ax = x_ax.cross(y_ax)
        if z_ax.length < 1e-9:
            z_ax = frame["z"].copy()
        z_ax.normalize()
        return {"x": x_ax, "y": y_ax, "z": z_ax}

    def resolve_cut_anchor(cut: dict, con: dict, info, support_info):
        member_end = cut_member_end(cut, con, info)
        frame = cut_local_frame(info, member_end)
        if not frame:
            return None, None
        inward = frame["x"].copy()
        axis_point = project_point_to_line(pt(con["at"]), info["start"], info["end"])
        support_half = projected_member_half_extent(support_info, inward)
        ref = cut.get("reference")
        if ref == "axis_start":
            anchor = info["start"].copy()
        elif ref == "axis_end":
            anchor = info["end"].copy()
        elif ref == "support_outer_edge":
            anchor = axis_point - inward * support_half
        elif ref == "support_centerline":
            anchor = axis_point
        else:
            anchor = axis_point + inward * support_half
        anchor += inward * float(cut.get("offset_mm", cut.get("x_from_support_edge_mm", 0)))
        return anchor, frame

    def notch_poly_2d(cut: dict, box_h: float, info, support_info, con: dict):
        overcut = 8.0
        bottom = -box_h / 2
        top = box_h / 2
        kind = cut.get("kind")
        if kind == "rect_notch":
            length = float(cut.get("length_mm", 0))
            depth = float(cut.get("depth_mm", 0))
            return (
                [(-overcut, top + overcut), (length + overcut, top + overcut), (length + overcut, top - depth - overcut), (-overcut, top - depth - overcut)]
                if cut.get("side") == "top"
                else [(-overcut, bottom - overcut), (length + overcut, bottom - overcut), (length + overcut, bottom + depth + overcut), (-overcut, bottom + depth + overcut)]
            )
        if kind == "bevel_notch":
            length = float(cut.get("length_mm", 0))
            depth = float(cut.get("depth_mm", 0))
            return (
                [(-overcut, top + overcut), (length + overcut, top + overcut), (-overcut, top - depth - overcut)]
                if cut.get("side") == "top"
                else [(-overcut, bottom - overcut), (length + overcut, bottom - overcut), (-overcut, bottom + depth + overcut)]
            )
        if kind == "end_bevel_cut":
            length = float(cut.get("length_mm", 0))
            return (
                [(-overcut, top + overcut), (length + overcut, top + overcut), (-overcut, bottom - overcut)]
                if cut.get("cut_from") == "top"
                else [(-overcut, bottom - overcut), (length + overcut, bottom - overcut), (-overcut, top + overcut)]
            )
        if kind != "birdsmouth_notch":
            return None
        frame = cut_local_frame(info, cut_member_end(cut, con, info))
        if not frame:
            return None
        support_normal = None
        if support_info and support_info["frame"]:
            support_normal = support_info["frame"]["y"].copy()
            if support_normal.dot(frame["y"]) < 0:
                support_normal.negate()
        if support_normal is None:
            support_normal = frame["y"].copy()
        plumb3 = support_normal - frame["z"] * support_normal.dot(frame["z"])
        if plumb3.length < 1e-8:
            plumb3 = frame["y"].copy()
        plumb2 = Vector((plumb3.dot(frame["x"]), plumb3.dot(frame["y"])))
        if plumb2.length < 1e-8:
            return None
        plumb2.normalize()
        if plumb2.y > 0 or (abs(plumb2.y) < 1e-6 and plumb2.x > 0):
            plumb2.negate()
        seat2 = Vector((-plumb2.y, plumb2.x))
        if seat2.length < 1e-8:
            return None
        seat2.normalize()
        if seat2.x > 0 or (abs(seat2.x) < 1e-6 and seat2.y < 0):
            seat2.negate()
        heel_y = bottom + float(cut.get("heel_depth_mm", 0))
        heel = Vector((0, heel_y))
        plumb_t = (bottom - heel_y) / plumb2.y
        plumb_end = Vector((plumb_t * plumb2.x, bottom))
        seat_end = Vector((seat2.x * float(cut.get("seat_length_mm", 0)), heel_y + seat2.y * float(cut.get("seat_length_mm", 0))))
        clipped = seat_end.copy()
        if clipped.y < bottom or clipped.y > top:
            bound = bottom if clipped.y < bottom else top
            t = (bound - heel_y) / seat2.y
            clipped = Vector((seat2.x * t, bound))
        seat_bottom = Vector((clipped.x, bottom))
        poly = (
            [(seat_bottom.x, seat_bottom.y), (plumb_end.x, plumb_end.y), (heel.x, heel.y), (clipped.x, clipped.y)]
            if abs(clipped.y - bottom) > 0.1
            else [(plumb_end.x, plumb_end.y), (heel.x, heel.y), (clipped.x, clipped.y)]
        )
        cx = sum(x for x, _ in poly) / len(poly)
        cy = sum(y for _, y in poly) / len(poly)
        expanded = []
        for x, y in poly:
            dx = x - cx
            dy = y - cy
            length = math.hypot(dx, dy)
            if length > 1e-9:
                x += dx / length * overcut
                y += dy / length * overcut
            expanded.append((x, y))
        return expanded

    def create_notch_cutter(name: str, cut: dict, con: dict, info, support_info):
        h_mm, b_mm = member_dims(info["member"])
        poly = notch_poly_2d(cut, h_mm, info, support_info, con)
        anchor, frame = resolve_cut_anchor(cut, con, info, support_info)
        if not poly or anchor is None or not frame:
            return None
        half_width = b_mm / 2 + 40
        front = [anchor + frame["x"] * x + frame["y"] * y + frame["z"] * half_width for x, y in poly]
        back = [anchor + frame["x"] * x + frame["y"] * y - frame["z"] * half_width for x, y in poly]
        verts = front + back
        n = len(poly)
        faces = [list(range(n)), list(range(2 * n - 1, n - 1, -1))]
        for i in range(n):
            faces.append([i, (i + 1) % n, n + (i + 1) % n, n + i])
        return create_mesh_object(name, verts, faces)

    def apply_notches(member_index: dict, geo: dict) -> None:
        cutter_material = material_principled("Notch cutters", (1, 0, 1, 0.15), alpha=0.15)
        for con in geometry_loader.expanded_connections(geo):
            if con.get("type") != "notched_over":
                continue
            cuts = con.get("cuts") or ([con["notch"]] if con.get("notch") else [])
            if not cuts or len(con.get("members", [])) < 1:
                continue
            info = member_index.get(con["members"][0])
            support_info = member_index.get(con["members"][1]) if len(con.get("members", [])) > 1 else None
            target = info.get("object") if info else None
            if target is None:
                continue
            for i, cut in enumerate(cuts):
                cutter = create_notch_cutter(f"cut.{con['id']}.{i}", cut, con, info, support_info)
                if cutter is None:
                    continue
                cutter.data.materials.append(cutter_material)
                cutter.hide_render = True
                modifier = target.modifiers.new(f"notch.{con['id']}.{i}", "BOOLEAN")
                modifier.operation = "DIFFERENCE"
                modifier.object = cutter
                if hasattr(modifier, "solver"):
                    modifier.solver = "EXACT"
                bpy.context.view_layer.objects.active = target
                target.select_set(True)
                try:
                    bpy.ops.object.modifier_apply(modifier=modifier.name)
                except Exception as exc:
                    print(f"WARNING: boolean notch failed for {con['id']}: {exc}")
                target.select_set(False)
                bpy.data.objects.remove(cutter, do_unlink=True)

    def surface_frame(points: list[Vector]):
        if len(points) < 3:
            return None
        origin = points[0]
        x_axis = None
        for point in points[1:]:
            candidate = point - origin
            if candidate.length > 1e-9:
                x_axis = candidate.normalized()
                break
        if x_axis is None:
            return None
        normal = None
        for point in points[2:]:
            candidate = x_axis.cross(point - origin)
            if candidate.length > 1e-9:
                normal = candidate.normalized()
                break
        if normal is None:
            return None
        return {"normal": normal}

    def create_surface(name: str, poly: list[dict], thickness_mm: float, mat):
        points = [pt(p) for p in poly]
        frame = surface_frame(points)
        if frame is None:
            return None
        if thickness_mm <= 0:
            return create_mesh_object(name, points, [list(range(len(points)))], mat)
        half = thickness_mm / 2
        front = [p + frame["normal"] * half for p in points]
        back = [p - frame["normal"] * half for p in points]
        verts = front + back
        n = len(points)
        faces = [list(range(n)), list(range(2 * n - 1, n - 1, -1))]
        for i in range(n):
            faces.append([i, (i + 1) % n, n + (i + 1) % n, n + i])
        return create_mesh_object(name, verts, faces, mat)

    def make_light_bounce_only(obj):
        obj.hide_render = False
        for attr, value in (
            ("visible_camera", False),
            ("visible_diffuse", True),
            ("visible_glossy", True),
            ("visible_transmission", True),
            ("visible_volume_scatter", False),
            ("visible_shadow", False),
        ):
            if hasattr(obj, attr):
                setattr(obj, attr, value)
        return obj

    def make_glossy_reflection_only(obj):
        obj.hide_render = False
        for attr, value in (
            ("visible_camera", False),
            ("visible_diffuse", False),
            ("visible_glossy", True),
            ("visible_transmission", True),
            ("visible_volume_scatter", False),
            ("visible_shadow", False),
        ):
            if hasattr(obj, attr):
                setattr(obj, attr, value)
        return obj

    def create_floor_bounce_plane(name: str, bounds: tuple[float, float, float, float], z_mm: float, mat):
        x_min, x_max, y_min, y_max = bounds
        poly = [
            {"x": x_min, "y": y_min, "z": z_mm},
            {"x": x_max, "y": y_min, "z": z_mm},
            {"x": x_max, "y": y_max, "z": z_mm},
            {"x": x_min, "y": y_max, "z": z_mm},
        ]
        return create_surface(name, poly, 0.0, mat)

    def create_quad_prism(name: str, top_corners: list[Vector], depth_dir: Vector, depth_mm: float, mat):
        bottom_corners = [corner + depth_dir.normalized() * depth_mm for corner in top_corners]
        verts = top_corners + bottom_corners
        faces = [
            [0, 1, 2, 3],
            [7, 6, 5, 4],
            [0, 4, 5, 1],
            [1, 5, 6, 2],
            [2, 6, 7, 3],
            [3, 7, 4, 0],
        ]
        return create_mesh_object(name, verts, faces, mat)

    def create_solar_panel_array(surface: dict, materials: dict, camera):
        poly = surface.get("polygon")
        count = surface.get("count") or {}
        if not poly or len(poly) != 4:
            return []
        nx = int(count.get("nx", 7))
        ny = int(count.get("ny", 2))
        bl, br, fr, fl = [pt(p) for p in poly]
        u_vec = br - bl
        v_vec = fl - bl
        if u_vec.length < 1e-9 or v_vec.length < 1e-9:
            return []
        u_axis = u_vec.normalized()
        v_axis = v_vec.normalized()
        normal = u_axis.cross(v_axis)
        if normal.length < 1e-9:
            return []
        normal.normalize()
        world_up = Vector((0, 1, 0))
        if normal.dot(world_up) < 0:
            normal.negate()
        top_offset = normal * (float(surface.get("thickness_mm", 0)) / 2)

        def on_panel(u: float, v: float) -> Vector:
            bottom = bl.lerp(br, u)
            top = fl.lerp(fr, u)
            return bottom.lerp(top, v) + top_offset

        def add_vertical_leg(name: str, a: Vector, b: Vector, inward: Vector, wall_mm: float, height_mm: float, mat):
            top_corners = [
                a + normal * 0.8,
                b + normal * 0.8,
                b + inward * wall_mm + normal * 0.8,
                a + inward * wall_mm + normal * 0.8,
            ]
            return create_quad_prism(name, top_corners, -normal, height_mm, mat)

        def add_bottom_lip(name: str, a: Vector, b: Vector, inward: Vector, lip_width_mm: float, height_mm: float, wall_mm: float, mat):
            lip_top = -normal * max(height_mm - wall_mm, 0)
            top_corners = [
                a + lip_top,
                b + lip_top,
                b + inward * lip_width_mm + lip_top,
                a + inward * lip_width_mm + lip_top,
            ]
            return create_quad_prism(name, top_corners, -normal, wall_mm, mat)

        def add_cell_line(name: str, a: Vector, b: Vector, width_axis: Vector, width_mm: float, surface_offset: Vector, mat):
            half = width_mm / 2
            verts = [
                a + width_axis * half + surface_offset,
                b + width_axis * half + surface_offset,
                b - width_axis * half + surface_offset,
                a - width_axis * half + surface_offset,
            ]
            return create_mesh_object(name, verts, [[0, 1, 2, 3]], mat)

        def add_cell_band(name: str, a: Vector, b: Vector, width_axis: Vector, width_mm: float, surface_offset: Vector, mat):
            half = width_mm / 2
            verts = [
                a + width_axis * half + surface_offset,
                b + width_axis * half + surface_offset,
                b - width_axis * half + surface_offset,
                a - width_axis * half + surface_offset,
            ]
            return create_mesh_object(name, verts, [[0, 1, 2, 3]], mat)

        objects = []
        for ix in range(nx):
            u0 = ix / nx
            u1 = (ix + 1) / nx
            for iy in range(ny):
                v0 = iy / ny
                v1 = (iy + 1) / ny
                p00 = on_panel(u0, v0)
                p10 = on_panel(u1, v0)
                p11 = on_panel(u1, v1)
                p01 = on_panel(u0, v1)

                frame_height = 30.0
                frame_wall = 4.0
                long_lip_width = 30.0
                short_lip_width = 15.0
                glass_thickness = 4.0
                glass_margin = frame_wall - 0.5

                glass = [
                    p00 + u_axis * glass_margin + v_axis * glass_margin,
                    p10 - u_axis * glass_margin + v_axis * glass_margin,
                    p11 - u_axis * glass_margin - v_axis * glass_margin,
                    p01 + u_axis * glass_margin - v_axis * glass_margin,
                ]
                objects.append(create_quad_prism(f"solar.glass.{ix}.{iy}", glass, -normal, glass_thickness, materials["solar"]))
                bottom_skin = [corner - normal * (glass_thickness + 0.9) for corner in glass]
                objects.append(create_mesh_object(
                    f"solar.bifacial.bottom.skin.{ix}.{iy}",
                    bottom_skin,
                    [[0, 3, 2, 1]],
                    materials["solar_bottom_glow"],
                ))

                frame_edges = [
                    (f"solar.frame.long.left.{ix}.{iy}", p00, p01, u_axis, long_lip_width),
                    (f"solar.frame.long.right.{ix}.{iy}", p10, p11, -u_axis, long_lip_width),
                    (f"solar.frame.short.front.{ix}.{iy}", p00, p10, v_axis, short_lip_width),
                    (f"solar.frame.short.back.{ix}.{iy}", p01, p11, -v_axis, short_lip_width),
                ]
                for name, a, b, inward, lip_width in frame_edges:
                    objects.append(add_vertical_leg(name + ".vertical", a, b, inward, frame_wall, frame_height, materials["solar_frame"]))
                    objects.append(add_bottom_lip(name + ".bottom_lip", a, b, inward, lip_width, frame_height, frame_wall, materials["solar_frame"]))

                cell_u0 = glass[0]
                cell_u1 = glass[1]
                cell_v1 = glass[3]
                top_cell_offset = normal * 1.2
                bottom_cell_offset = -normal * (glass_thickness + 1.2)
                for cell_u in range(1, 6):
                    t = cell_u / 6
                    a = cell_u0.lerp(cell_u1, t)
                    b = glass[3].lerp(glass[2], t)
                    objects.append(add_cell_line(f"solar.cell.top.u.{ix}.{iy}.{cell_u}", a, b, u_axis, 2.0, top_cell_offset, materials["solar_cell_grid"]))
                    objects.append(add_cell_band(f"solar.cell.bottom.u.{ix}.{iy}.{cell_u}", a, b, u_axis, 3.2, bottom_cell_offset, materials["solar_bottom_cell_grid"]))
                for cell_v in range(1, 10):
                    t = cell_v / 10
                    a = cell_u0.lerp(cell_v1, t)
                    b = glass[1].lerp(glass[2], t)
                    objects.append(add_cell_line(f"solar.cell.top.v.{ix}.{iy}.{cell_v}", a, b, v_axis, 2.0, top_cell_offset, materials["solar_cell_grid"]))
                    objects.append(add_cell_band(f"solar.cell.bottom.v.{ix}.{iy}.{cell_v}", a, b, v_axis, 3.2, bottom_cell_offset, materials["solar_bottom_cell_grid"]))
        return objects

    def add_photo_plane(calibration: dict, camera, photo_path: Path, mat):
        projection = calibration["camera"]["projection_matrix"]
        fov_y = 2 * math.atan(1 / float(projection[5]))
        aspect = float(calibration["camera"]["aspect"])
        distance = 60000.0
        camera_matrix = camera.matrix_world
        right = (camera_matrix.to_3x3() @ Vector((1, 0, 0))).normalized()
        up = (camera_matrix.to_3x3() @ Vector((0, 1, 0))).normalized()
        forward = (camera_matrix.to_3x3() @ Vector((0, 0, -1))).normalized()
        center = camera.location + forward * distance
        height = 2 * distance * math.tan(fov_y / 2)
        width = height * aspect
        verts = [
            center - right * width / 2 - up * height / 2,
            center + right * width / 2 - up * height / 2,
            center + right * width / 2 + up * height / 2,
            center - right * width / 2 + up * height / 2,
        ]
        obj = create_mesh_object("photo.background", verts, [[0, 1, 2, 3]], mat)
        uv = obj.data.uv_layers.new(name="UVMap")
        uv_values = [(0, 0), (1, 0), (1, 1), (0, 1)]
        for loop_idx, uv_coord in enumerate(uv_values):
            uv.data[loop_idx].uv = uv_coord
        obj.hide_select = True
        return obj

    def foreground_mask_path(output_path: Path) -> Path:
        stem = output_path.with_suffix("")
        return stem.with_name(stem.name + "_foreground_mask.png")

    def foreground_extra_mask_path(output_path: Path) -> Path:
        stem = output_path.with_suffix("")
        return stem.with_name(stem.name + "_foreground_extra_mask.png")

    def create_foreground_edge_band(mat):
        if args.foreground_edge_top_mm <= args.foreground_edge_bottom_mm:
            return None
        y = 5275
        x_min = -80
        x_max = 7280
        z_min = args.foreground_edge_bottom_mm
        z_max = args.foreground_edge_top_mm
        verts = [
            pt((x_min, y, z_min)),
            pt((x_max, y, z_min)),
            pt((x_max, y, z_max)),
            pt((x_min, y, z_max)),
        ]
        obj = create_mesh_object("foreground.occluder.terrace_front_edge", verts, [[0, 1, 2, 3]], mat)
        obj.hide_render = True
        return obj

    def create_foreground_occluders(mat) -> list:
        if not args.foreground_occlusion:
            return []
        occluder_ids = {
            item.strip()
            for item in args.foreground_occluder_ids.split(",")
            if item.strip()
        }
        foreground_geo = geometry_loader.load(args.foreground_occlusion_geometry + ".json")
        objects = []
        found = set()
        for group in ("columns", "beams"):
            for member in geometry_loader.expanded_members(foreground_geo, group):
                member_id = member.get("id")
                if member_id not in occluder_ids:
                    continue
                obj = create_box_member(f"foreground.occluder.{member_id}", group, member, mat)
                if obj:
                    obj.hide_render = True
                    objects.append(obj)
                    found.add(member_id)
        missing = sorted(occluder_ids - found)
        if missing:
            print("WARNING: foreground occluder ids not found:", ", ".join(missing))
        for geometry_name in (
            item.strip()
            for item in args.foreground_extra_occlusion_geometries.split(",")
            if item.strip()
        ):
            extra_geo = geometry_loader.load(geometry_name + ".json")
            extra_count = 0
            base_member_margin = max(0.0, float(args.foreground_extra_member_margin_mm))
            rafter_margin = max(0.0, float(args.foreground_extra_rafter_margin_mm))
            for group in ("columns", "beams", "rafters", "purlins"):
                for member in geometry_loader.expanded_members(extra_geo, group):
                    member_margin = base_member_margin
                    if group in {"rafters", "purlins"}:
                        member_margin += rafter_margin
                    obj = create_box_member(
                        f"foreground.occluder.{geometry_name}.{member['id']}",
                        group,
                        member,
                        mat,
                        margin_mm=member_margin,
                    )
                    if obj:
                        obj.hide_render = True
                        obj["_foreground_extra_occluder"] = True
                        objects.append(obj)
                        extra_count += 1
            for surface in extra_geo.get("surfaces", []):
                poly = surface.get("polygon")
                if not poly:
                    continue
                obj = create_surface(
                    f"foreground.occluder.{geometry_name}.{surface.get('id', 'surface')}",
                    poly,
                    float(surface.get("thickness_mm", 0)),
                    mat,
                )
                if obj:
                    obj.hide_render = True
                    obj["_foreground_extra_occluder"] = True
                    objects.append(obj)
                    extra_count += 1
            print(f"Foreground occlusion: added {extra_count} objects from {geometry_name}.json")
        edge_band = create_foreground_edge_band(mat)
        if edge_band:
            objects.append(edge_band)
        return objects

    def column_color_sample_mask_path(output_path: Path) -> Path:
        stem = output_path.with_suffix("")
        return stem.with_name(stem.name + "_column_color_sample_mask.png")

    def create_column_color_sample_objects(mat) -> list:
        ids = {
            item.strip()
            for item in args.column_color_sample_ids.split(",")
            if item.strip()
        }
        if not ids:
            return []
        sample_geo = geometry_loader.load(args.column_color_sample_geometry + ".json")
        objects = []
        found = set()
        for member in geometry_loader.expanded_members(sample_geo, "columns"):
            member_id = member.get("id")
            if member_id not in ids:
                continue
            obj = create_box_member(f"column.color.sample.{member_id}", "columns", member, mat)
            if obj:
                obj.hide_render = True
                objects.append(obj)
                found.add(member_id)
        missing = sorted(ids - found)
        if missing:
            print("WARNING: column color sample ids not found:", ", ".join(missing))
        return objects

    def render_column_color_sample_mask(path: Path, photo_obj, sample_objects: list) -> Path | None:
        if not sample_objects:
            return None
        scene = bpy.context.scene
        original_filepath = scene.render.filepath
        original_film_transparent = scene.render.film_transparent
        original_world_color = tuple(scene.world.color)
        original_view_transform = scene.view_settings.view_transform
        original_look = scene.view_settings.look
        original_use_nodes = scene.use_nodes
        original_use_freestyle = scene.render.use_freestyle
        original_samples = scene.cycles.samples if scene.render.engine == "CYCLES" else None
        all_objects = [photo_obj, *sample_objects]
        original_hide_render = {obj.name: obj.hide_render for obj in all_objects}
        white = material_emission_color("Column color sample white", (1, 1, 1, 1), 1.0)
        try:
            photo_obj.hide_render = True
            for obj in sample_objects:
                set_object_material(obj, white)
                obj.hide_render = False
            if scene.render.engine == "CYCLES":
                scene.cycles.samples = max(1, int(args.guide_samples))
                scene.cycles.use_denoising = False
            scene.use_nodes = False
            scene.render.film_transparent = False
            scene.world.color = (0, 0, 0)
            scene.view_settings.view_transform = "Raw"
            scene.view_settings.look = "None"
            set_freestyle(False)
            render_still_to(path)
            return path
        finally:
            scene.use_nodes = original_use_nodes
            scene.render.use_freestyle = original_use_freestyle
            scene.render.film_transparent = original_film_transparent
            scene.world.color = original_world_color
            scene.view_settings.view_transform = original_view_transform
            scene.view_settings.look = original_look
            scene.render.filepath = original_filepath
            if original_samples is not None:
                scene.cycles.samples = original_samples
                scene.cycles.use_denoising = True
            for obj in all_objects:
                if obj.name in original_hide_render:
                    obj.hide_render = original_hide_render[obj.name]

    def sample_photo_color(photo_path: Path, mask_path: Path) -> tuple[tuple[float, float, float, float], int] | None:
        from PIL import Image, ImageFilter

        mask = Image.open(mask_path).convert("L")
        mask = mask.point(lambda value: 255 if value >= 32 else 0)
        erode_px = min(
            max(0, int(args.column_color_sample_erode_px)),
            max(0, min(mask.size) // 400),
        )
        if erode_px:
            mask = mask.filter(ImageFilter.MinFilter(erode_px * 2 + 1))
        original = Image.open(photo_path).convert("RGB")
        if original.size != mask.size:
            original = original.resize(mask.size, Image.Resampling.LANCZOS)

        mask_data = list(mask.getdata())
        photo_data = list(original.getdata())
        candidate_count = sum(1 for value in mask_data if value >= 128)
        if candidate_count < max(1, int(args.column_color_match_min_pixels)):
            return None
        stride = max(1, candidate_count // 200_000)
        pixels = []
        seen = 0
        for value, rgb in zip(mask_data, photo_data):
            if value < 128:
                continue
            seen += 1
            if seen % stride:
                continue
            r, g, b = rgb
            luma = 0.2126 * r + 0.7152 * g + 0.0722 * b
            if 18 <= luma <= 248:
                pixels.append((r, g, b))
        if len(pixels) < max(1, int(args.column_color_match_min_pixels)):
            return None

        channels = []
        for idx in range(3):
            values = sorted(pixel[idx] for pixel in pixels)
            channels.append(values[len(values) // 2] / 255)
        return (channels[0], channels[1], channels[2], 1.0), len(pixels)

    def remove_objects(objects: list) -> None:
        for obj in objects:
            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except ReferenceError:
                pass

    def resolve_column_color(photo_path: Path, photo_obj, output_path: Path) -> tuple[float, float, float, float]:
        manual = parse_rgb_color(args.column_color)
        if manual:
            print(f"Column color: manual {rgb_to_hex(manual)}")
            return manual
        if not args.column_color_match:
            print(f"Column color: default {rgb_to_hex(DEFAULT_COLUMN_COLOR)}")
            return DEFAULT_COLUMN_COLOR

        sample_objects = create_column_color_sample_objects(material_emission_color("Column color sample", (1, 1, 1, 1), 1.0))
        mask_path = column_color_sample_mask_path(output_path)
        try:
            rendered_mask = render_column_color_sample_mask(mask_path, photo_obj, sample_objects)
            if rendered_mask:
                sampled = sample_photo_color(photo_path, rendered_mask)
                if sampled:
                    sampled_color, pixel_count = sampled
                    color = blend_rgb(DEFAULT_COLUMN_COLOR, sampled_color, args.column_color_match_strength)
                    print(
                        f"Column color: sampled {rgb_to_hex(sampled_color)} from {pixel_count} px "
                        f"-> material {rgb_to_hex(color)}"
                    )
                    return color
        finally:
            remove_objects(sample_objects)
            try:
                if mask_path.exists():
                    mask_path.unlink()
            except OSError:
                pass
        print(f"WARNING: column color sampling failed; using default {rgb_to_hex(DEFAULT_COLUMN_COLOR)}")
        return DEFAULT_COLUMN_COLOR

    def postprocess_foreground_mask(mask_path: Path) -> None:
        from PIL import Image, ImageFilter

        mask = Image.open(mask_path).convert("L")
        mask = mask.point(lambda value: 255 if value >= 32 else 0)
        grow_px = max(0, int(args.foreground_mask_grow_px))
        if grow_px:
            mask = mask.filter(ImageFilter.MaxFilter(grow_px * 2 + 1))
        if args.foreground_mask_blur_px > 0:
            mask = mask.filter(ImageFilter.GaussianBlur(float(args.foreground_mask_blur_px)))
        mask.save(mask_path)

    def shifted_mask(mask, dx: int, dy: int):
        from PIL import Image

        width, height = mask.size
        src = (
            max(0, -dx),
            max(0, -dy),
            min(width, width - dx),
            min(height, height - dy),
        )
        if src[0] >= src[2] or src[1] >= src[3]:
            return Image.new("L", mask.size, 0)
        out = Image.new("L", mask.size, 0)
        out.paste(mask.crop(src), (max(0, dx), max(0, dy)))
        return out

    def grow_mask_directional(mask, left: int, right: int, up: int, down: int):
        from PIL import ImageChops

        grown = mask
        for dx in range(1, max(0, left) + 1):
            grown = ImageChops.lighter(grown, shifted_mask(mask, -dx, 0))
        for dx in range(1, max(0, right) + 1):
            grown = ImageChops.lighter(grown, shifted_mask(mask, dx, 0))

        horizontal = grown
        for dy in range(1, max(0, up) + 1):
            grown = ImageChops.lighter(grown, shifted_mask(horizontal, 0, -dy))
        for dy in range(1, max(0, down) + 1):
            grown = ImageChops.lighter(grown, shifted_mask(horizontal, 0, dy))
        return grown

    def postprocess_extra_foreground_mask(mask_path: Path) -> None:
        from PIL import Image, ImageFilter

        mask = Image.open(mask_path).convert("L")
        mask = mask.point(lambda value: 255 if value >= 32 else 0)
        mask = grow_mask_directional(
            mask,
            args.foreground_extra_mask_grow_left_px,
            args.foreground_extra_mask_grow_right_px,
            args.foreground_extra_mask_grow_up_px,
            args.foreground_extra_mask_grow_down_px,
        )
        if args.foreground_extra_mask_blur_px > 0:
            mask = mask.filter(ImageFilter.GaussianBlur(float(args.foreground_extra_mask_blur_px)))
        mask.save(mask_path)

    def merge_foreground_masks(base_path: Path, extra_path: Path) -> None:
        from PIL import Image, ImageChops

        base = Image.open(base_path).convert("L")
        extra = Image.open(extra_path).convert("L")
        if extra.size != base.size:
            extra = extra.resize(base.size, Image.Resampling.LANCZOS)
        ImageChops.lighter(base, extra).save(base_path)

    def render_foreground_mask(path: Path, photo_obj, render_objects: list, foreground_objects: list) -> Path | None:
        if not foreground_objects:
            return None

        scene = bpy.context.scene
        original_filepath = scene.render.filepath
        original_film_transparent = scene.render.film_transparent
        original_world_color = tuple(scene.world.color)
        original_view_transform = scene.view_settings.view_transform
        original_look = scene.view_settings.look
        original_use_nodes = scene.use_nodes
        original_use_freestyle = scene.render.use_freestyle
        original_samples = scene.cycles.samples if scene.render.engine == "CYCLES" else None
        all_objects = [photo_obj, *render_objects, *foreground_objects]
        original_hide_render = {obj.name: obj.hide_render for obj in all_objects}
        original_materials = {
            obj.name: [slot.material for slot in obj.material_slots]
            for obj in [*render_objects, *foreground_objects]
        }
        black = material_emission_color("Foreground occlusion depth blockers", (0, 0, 0, 1), 1.0)
        white = material_emission_color("Foreground occlusion white", (1, 1, 1, 1), 1.0)

        try:
            photo_obj.hide_render = True
            for obj in render_objects:
                set_object_material(obj, black)
                obj.hide_render = False
            for obj in foreground_objects:
                set_object_material(obj, white)
                obj.hide_render = False

            if scene.render.engine == "CYCLES":
                scene.cycles.samples = max(1, int(args.guide_samples))
                scene.cycles.use_denoising = False
            scene.use_nodes = False
            scene.render.film_transparent = False
            scene.world.color = (0, 0, 0)
            scene.view_settings.view_transform = "Raw"
            scene.view_settings.look = "None"
            set_freestyle(False)
            render_still_to(path)
            postprocess_foreground_mask(path)
            extra_objects = [
                obj
                for obj in foreground_objects
                if bool(obj.get("_foreground_extra_occluder", False))
            ]
            needs_extra_mask = (
                extra_objects
                and (
                    args.foreground_extra_mask_grow_left_px
                    or args.foreground_extra_mask_grow_right_px
                    or args.foreground_extra_mask_grow_up_px
                    or args.foreground_extra_mask_grow_down_px
                    or args.foreground_extra_mask_blur_px > 0
                )
            )
            if needs_extra_mask:
                for obj in render_objects:
                    obj.hide_render = False
                for obj in foreground_objects:
                    obj.hide_render = obj not in extra_objects
                extra_path = foreground_extra_mask_path(path)
                render_still_to(extra_path)
                postprocess_extra_foreground_mask(extra_path)
                merge_foreground_masks(path, extra_path)
            return path
        finally:
            scene.use_nodes = original_use_nodes
            scene.render.use_freestyle = original_use_freestyle
            scene.render.film_transparent = original_film_transparent
            scene.world.color = original_world_color
            scene.view_settings.view_transform = original_view_transform
            scene.view_settings.look = original_look
            scene.render.filepath = original_filepath
            if original_samples is not None:
                scene.cycles.samples = original_samples
                scene.cycles.use_denoising = True
            for obj in all_objects:
                if obj.name in original_hide_render:
                    obj.hide_render = original_hide_render[obj.name]
            for obj in [*render_objects, *foreground_objects]:
                obj.data.materials.clear()
                for mat in original_materials.get(obj.name, []):
                    if mat:
                        obj.data.materials.append(mat)

    def apply_foreground_occlusion_to_image(image_path: Path, photo_path: Path, mask_path: Path | None) -> None:
        if mask_path is None:
            return
        from PIL import Image

        rendered = Image.open(image_path).convert("RGBA")
        original = Image.open(photo_path).convert("RGBA")
        if original.size != rendered.size:
            original = original.resize(rendered.size, Image.Resampling.LANCZOS)
        mask = Image.open(mask_path).convert("L")
        if mask.size != rendered.size:
            mask = mask.resize(rendered.size, Image.Resampling.LANCZOS)
        composited = Image.composite(original, rendered, mask)
        composited.save(image_path)
        print(f"Applied foreground occlusion mask {mask_path} to {image_path}")

    def render_geometry_bounds(render_objects: list) -> tuple[float, float, float, float, float, float]:
        xs = []
        ys = []
        zs = []
        for obj in render_objects:
            if obj.type != "MESH":
                continue
            for corner in obj.bound_box:
                world = obj.matrix_world @ Vector(corner)
                xs.append(world.x)
                ys.append(world.z)
                zs.append(world.y)
        if not xs or not ys or not zs:
            return -500.0, 7700.0, -500.0, 5900.0, -500.0, 5500.0
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)

    def render_geometry_xy_bounds(render_objects: list, padding_mm: float | None = None) -> tuple[float, float, float, float]:
        x_min, x_max, y_min, y_max, _, _ = render_geometry_bounds(render_objects)
        padding = max(0.0, float(args.bounce_floor_padding_mm if padding_mm is None else padding_mm))
        return x_min - padding, x_max + padding, y_min - padding, y_max + padding

    def create_bounce_surfaces(geo: dict, render_objects: list, materials: dict) -> list:
        if not args.bounce_surfaces:
            return []
        objects = []
        for surface in geo.get("reference_surfaces", []):
            poly = surface.get("polygon")
            if not poly:
                continue
            surface_type = surface.get("type", "")
            if surface_type in {"building_wall", "retaining_wall"}:
                mat = materials["bounce_wall"]
            elif surface_type in {"terrace_floor", "floor", "ground"}:
                mat = materials["bounce_floor"]
            else:
                continue
            obj = create_surface(f"bounce.{surface.get('id', surface_type)}", poly, 0.0, mat)
            if obj:
                make_light_bounce_only(obj)
                objects.append(obj)

        if not any(obj.name.startswith("bounce.ref.terrace_floor") for obj in objects):
            obj = create_floor_bounce_plane(
                "bounce.auto_terrace_floor",
                render_geometry_xy_bounds(render_objects),
                float(args.bounce_floor_z_mm),
                materials["bounce_floor"],
            )
            if obj:
                make_light_bounce_only(obj)
                objects.append(obj)

        print(f"Bounce surfaces: added {len(objects)} camera-invisible reflectors.")
        return objects

    def create_solar_sky_reflection_surfaces(render_objects: list, materials: dict) -> list:
        if not args.solar_sky_reflection:
            return []
        x_min, x_max, y_min, y_max = render_geometry_xy_bounds(
            render_objects,
            float(args.solar_sky_reflection_padding_mm),
        )
        _, _, _, _, _, z_max = render_geometry_bounds(render_objects)
        z = z_max + max(0.0, float(args.solar_sky_reflection_height_mm))
        obj = create_surface(
            "reflection.sky.card",
            [
                {"x": x_min, "y": y_min, "z": z},
                {"x": x_max, "y": y_min, "z": z},
                {"x": x_max, "y": y_max, "z": z},
                {"x": x_min, "y": y_max, "z": z},
            ],
            0.0,
            materials["solar_sky_reflection"],
        )
        if not obj:
            return []
        make_glossy_reflection_only(obj)
        print("Solar sky reflection: added camera-invisible glossy sky card.")
        return [obj]

    def setup_camera(calibration: dict):
        cam_data = bpy.data.cameras.new("Viewer camera")
        cam = bpy.data.objects.new("Viewer camera", cam_data)
        bpy.context.collection.objects.link(cam)
        cam.matrix_world = three_matrix(calibration["camera"]["matrix_world"])
        projection = calibration["camera"]["projection_matrix"]
        cam_data.type = "PERSP"
        cam_data.sensor_fit = "VERTICAL"
        cam_data.angle = 2 * math.atan(1 / float(projection[5]))
        cam_data.clip_start = float(calibration["camera"].get("near", 1))
        cam_data.clip_end = float(calibration["camera"].get("far", 200000))
        bpy.context.scene.camera = cam
        return cam

    def sun_ray_direction_from_camera(camera) -> Vector:
        up = Vector((0, 1, 0))
        forward = camera.matrix_world.to_3x3() @ Vector((0, 0, -1))
        forward_h = forward - up * forward.dot(up)
        if forward_h.length < 1e-9:
            forward_h = Vector((0, 0, -1))
        forward_h.normalize()

        elevation = math.radians(args.sun_elevation_deg)
        if args.sun_facing_camera:
            scene_to_sun = forward_h * math.cos(elevation) + up * math.sin(elevation)
            scene_to_sun.normalize()
            return -scene_to_sun, "camera-facing"

        true_north_xy = geometry_true_north_xy(geo)
        if true_north_xy:
            north = Vector((true_north_xy[0], 0, true_north_xy[1])).normalized()
            east = Vector((-true_north_xy[1], 0, true_north_xy[0])).normalized()
            azimuth = math.radians(args.sun_azimuth_deg)
            scene_to_sun = (
                east * math.sin(azimuth) * math.cos(elevation)
                + north * math.cos(azimuth) * math.cos(elevation)
                + up * math.sin(elevation)
            )
            scene_to_sun.normalize()
            return -scene_to_sun, "geometry-north"

        # Camera heading from EXIF maps this calibrated horizontal view direction
        # to true north/east, so true sun azimuth can be converted to model space.
        if args.camera_heading_deg is None:
            args.camera_heading_deg = resolve_camera_heading_deg(calibration, None)
        right_h = forward_h.cross(up)
        if right_h.length < 1e-9:
            right_h = Vector((1, 0, 0))
        right_h.normalize()

        heading = math.radians(args.camera_heading_deg)
        east = forward_h * math.sin(heading) + right_h * math.cos(heading)
        north = forward_h * math.cos(heading) - right_h * math.sin(heading)
        east.normalize()
        north.normalize()

        azimuth = math.radians(args.sun_azimuth_deg)
        scene_to_sun = (
            east * math.sin(azimuth) * math.cos(elevation)
            + north * math.cos(azimuth) * math.cos(elevation)
            + up * math.sin(elevation)
        )
        scene_to_sun.normalize()
        return -scene_to_sun, "camera-heading"

    def setup_lighting(camera) -> None:
        world = bpy.context.scene.world or bpy.data.worlds.new("World")
        bpy.context.scene.world = world
        world.color = (0.03, 0.035, 0.045)

        bpy.ops.object.light_add(type="SUN", location=(0, 0, 0))
        sun = bpy.context.object
        sun.name = "Sun"
        sun.data.energy = args.sun_energy
        sun.data.angle = math.radians(args.sun_angle_deg)
        sun.data.color = args.sun_color_rgb[:3]
        direction, sun_mode = sun_ray_direction_from_camera(camera)
        sun.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
        camera_heading = "n/a" if args.camera_heading_deg is None else f"{args.camera_heading_deg:.3f} deg"
        print(
            "Sun:",
            f"azimuth={args.sun_azimuth_deg:.3f} deg",
            f"elevation={args.sun_elevation_deg:.3f} deg",
            f"source={args.sun_position_source}",
            f"color={rgb_to_hex(args.sun_color_rgb)}",
            f"color_source={args.sun_color_source}",
            f"camera_heading={camera_heading}",
            f"mode={sun_mode}",
            f"ray_direction=({direction.x:.4f}, {direction.y:.4f}, {direction.z:.4f})",
        )

        bpy.ops.object.light_add(type="AREA", location=(-2500, 5000, 9000))
        area = bpy.context.object
        area.name = "Soft sky fill"
        area.data.energy = args.sky_fill_energy
        area.data.size = 6500
        area.data.color = (0.78, 0.86, 1.0)

    def setup_render(calibration: dict) -> None:
        scene = bpy.context.scene
        natural = calibration["image"]["natural_size_px"]
        scene.render.resolution_x = max(1, int(round(natural["width"] * args.scale)))
        scene.render.resolution_y = max(1, int(round(natural["height"] * args.scale)))
        scene.render.resolution_percentage = 100
        scene.render.filepath = str((ROOT / args.output).resolve())
        scene.render.image_settings.file_format = "PNG"
        scene.render.image_settings.color_mode = "RGBA"
        scene.view_settings.view_transform = "Standard"
        scene.view_settings.look = "Medium High Contrast"
        scene.view_settings.exposure = 0
        scene.view_settings.gamma = 1
        if args.engine == "CYCLES":
            scene.render.engine = "CYCLES"
            scene.cycles.samples = args.samples
            scene.cycles.use_denoising = True
            scene.cycles.max_bounces = 8
            scene.cycles.diffuse_bounces = 4
            scene.cycles.transparent_max_bounces = 8
        else:
            try:
                scene.render.engine = "BLENDER_EEVEE_NEXT"
            except TypeError:
                scene.render.engine = "BLENDER_EEVEE"

    def set_object_material(obj, mat) -> None:
        obj.data.materials.clear()
        obj.data.materials.append(mat)

    def render_color_for_item(item: dict | None) -> tuple[float, float, float, float] | None:
        if not isinstance(item, dict):
            return None
        color = item.get("render_color")
        if not color:
            return None
        return parse_rgb_color(str(color))

    def member_render_color(member: dict) -> tuple[float, float, float, float] | None:
        return render_color_for_item(member) or render_color_for_item(member.get("profile"))

    def guide_output_paths(output_path: Path) -> dict[str, Path]:
        stem = output_path.with_suffix("")
        return {
            "structure": stem.with_name(stem.name + "_structure.png"),
            "mask": stem.with_name(stem.name + "_mask.png"),
            "mask_soft": stem.with_name(stem.name + "_mask_soft.png"),
            "lineart": stem.with_name(stem.name + "_lineart.png"),
            "depth": stem.with_name(stem.name + "_depth.png"),
        }

    def render_still_to(path: Path) -> None:
        bpy.context.scene.render.filepath = str(path.resolve())
        bpy.ops.render.render(write_still=True)
        print(f"Rendered {path}")

    def set_freestyle(enabled: bool) -> None:
        scene = bpy.context.scene
        scene.render.use_freestyle = enabled
        if not enabled:
            return
        view_layer = scene.view_layers[0]
        view_layer.use_freestyle = True
        line_sets = view_layer.freestyle_settings.linesets
        line_set = line_sets[0] if line_sets else line_sets.new("AI lineart")
        line_style = line_set.linestyle
        line_style.thickness = 2.4
        line_style.color = (0, 0, 0)

    def load_foreground_mask(mask_path: Path | None, size: tuple[int, int]):
        if mask_path is None:
            return None
        from PIL import Image

        mask = Image.open(mask_path).convert("L")
        if mask.size != size:
            mask = mask.resize(size, Image.Resampling.LANCZOS)
        return mask.point(lambda value: 255 if value > 1 else 0)

    def postprocess_ai_guides(paths: dict[str, Path], foreground_mask_path_value: Path | None = None) -> None:
        from PIL import Image, ImageChops, ImageFilter

        mask = Image.open(paths["mask"]).convert("L")
        mask = mask.point(lambda value: 255 if value >= 96 else 0)
        grow_px = max(0, int(args.mask_grow_px))
        if grow_px:
            mask = mask.filter(ImageFilter.MaxFilter(grow_px * 2 + 1))
        foreground_mask = load_foreground_mask(foreground_mask_path_value, mask.size)
        if foreground_mask is not None:
            mask.paste(0, mask=foreground_mask)
        mask_rgb = Image.merge("RGB", (mask, mask, mask))
        mask_rgb.save(paths["mask"])

        soft = mask
        if args.mask_blur_px > 0:
            soft = soft.filter(ImageFilter.GaussianBlur(float(args.mask_blur_px)))
        Image.merge("RGB", (soft, soft, soft)).save(paths["mask_soft"])

        rendered_lineart = Image.open(paths["lineart"]).convert("L")
        # Convert Blender output to a ControlNet-friendly black-line-on-white
        # guide. Some render backends keep the world/background dark here, so
        # derive edges from the rendered structure fill when needed.
        corner_points = [
            (0, 0),
            (rendered_lineart.width - 1, 0),
            (0, rendered_lineart.height - 1),
            (rendered_lineart.width - 1, rendered_lineart.height - 1),
        ]
        corner_mean = sum(rendered_lineart.getpixel(point) for point in corner_points) / len(corner_points)
        if corner_mean < 128:
            structure_fill = rendered_lineart.point(lambda value: 255 if value >= args.line_threshold else 0)
            edge_mask = structure_fill.filter(ImageFilter.FIND_EDGES).point(lambda value: 255 if value >= 16 else 0)
            edge_mask = edge_mask.filter(ImageFilter.MaxFilter(3))
        else:
            edge_mask = rendered_lineart.point(lambda value: 255 if value < args.line_threshold else 0)
        mask_outline = ImageChops.difference(
            mask.filter(ImageFilter.MaxFilter(5)),
            mask.filter(ImageFilter.MinFilter(5)),
        ).point(lambda value: 255 if value else 0)
        edge_mask = ImageChops.lighter(edge_mask, mask_outline)
        lineart = Image.new("L", rendered_lineart.size, 255)
        lineart.paste(0, mask=edge_mask)
        if foreground_mask is not None:
            lineart.paste(255, mask=foreground_mask)
        Image.merge("RGB", (lineart, lineart, lineart)).save(paths["lineart"])

        depth = Image.open(paths["depth"]).convert("L")
        # Keep depth grayscale but force any transparent/near-black color-managed
        # background to true black for depth-control tools.
        depth = depth.point(lambda value: 0 if value < 4 else value)
        if foreground_mask is not None:
            depth.paste(0, mask=foreground_mask)
        Image.merge("RGB", (depth, depth, depth)).save(paths["depth"])

        if foreground_mask is not None and paths["structure"].exists():
            structure = Image.open(paths["structure"]).convert("RGBA")
            structure_mask = foreground_mask
            if structure_mask.size != structure.size:
                structure_mask = structure_mask.resize(structure.size, Image.Resampling.LANCZOS)
            alpha = structure.getchannel("A")
            alpha.paste(0, mask=structure_mask)
            structure.putalpha(alpha)
            structure.save(paths["structure"])

    def render_ai_guides(
        calibration: dict,
        camera,
        photo_obj,
        render_objects: list,
        output_path: Path,
        foreground_mask_path_value: Path | None = None,
    ) -> None:
        scene = bpy.context.scene
        paths = guide_output_paths(output_path)
        original_filepath = scene.render.filepath
        original_film_transparent = scene.render.film_transparent
        original_world_color = tuple(scene.world.color)
        original_view_transform = scene.view_settings.view_transform
        original_look = scene.view_settings.look
        original_use_nodes = scene.use_nodes
        original_use_freestyle = scene.render.use_freestyle
        original_samples = scene.cycles.samples if scene.render.engine == "CYCLES" else None
        original_materials = {
            obj.name: [slot.material for slot in obj.material_slots]
            for obj in render_objects
        }
        original_hide_render = {obj.name: obj.hide_render for obj in [photo_obj, *render_objects]}

        white = material_emission_color("AI guide white", (1, 1, 1, 1), 1.0)
        if scene.render.engine == "CYCLES":
            scene.cycles.samples = args.guide_samples
            scene.cycles.use_denoising = False
        scene.view_settings.view_transform = "Raw"
        scene.view_settings.look = "None"

        try:
            photo_obj.hide_render = True

            # Real material structure with transparent background.
            scene.render.film_transparent = True
            scene.world.color = (0, 0, 0)
            set_freestyle(False)
            render_still_to(paths["structure"])

            # Binary inpaint mask: white = area AI may modify.
            scene.render.film_transparent = False
            scene.world.color = (0, 0, 0)
            for obj in render_objects:
                set_object_material(obj, white)
                obj.hide_render = False
            set_freestyle(False)
            render_still_to(paths["mask"])

            # Lineart / edge control on white background.
            scene.world.color = (1, 1, 1)
            set_freestyle(True)
            render_still_to(paths["lineart"])

            # Depth control for the new structure only. Nearer objects are white.
            scene.render.use_freestyle = False
            scene.world.color = (0, 0, 0)
            scene.view_layers[0].use_pass_z = True
            depths = []
            inv = camera.matrix_world.inverted()
            for obj in render_objects:
                if obj.type != "MESH":
                    continue
                for corner in obj.bound_box:
                    depth = -(inv @ (obj.matrix_world @ Vector(corner))).z
                    if depth > 0:
                        depths.append(depth)
            if depths:
                near = min(depths)
                far = max(depths)
                pad = max((far - near) * 0.08, 100.0)
                near = max(0.01, near - pad)
                far += pad
                scene.use_nodes = True
                tree = scene.node_tree
                tree.nodes.clear()
                render_layers = tree.nodes.new("CompositorNodeRLayers")
                map_range = tree.nodes.new("CompositorNodeMapRange")
                map_range.inputs[1].default_value = near
                map_range.inputs[2].default_value = far
                map_range.inputs[3].default_value = 1.0
                map_range.inputs[4].default_value = 0.0
                map_range.use_clamp = True
                composite = tree.nodes.new("CompositorNodeComposite")
                z_output = render_layers.outputs.get("Depth") or render_layers.outputs.get("Z")
                if z_output is None:
                    raise RuntimeError("Render Layers node has no Depth/Z output")
                tree.links.new(z_output, map_range.inputs[0])
                tree.links.new(map_range.outputs[0], composite.inputs[0])
                render_still_to(paths["depth"])
            postprocess_ai_guides(paths, foreground_mask_path_value)
        finally:
            scene.use_nodes = original_use_nodes
            scene.render.use_freestyle = original_use_freestyle
            scene.render.film_transparent = original_film_transparent
            scene.world.color = original_world_color
            scene.view_settings.view_transform = original_view_transform
            scene.view_settings.look = original_look
            scene.render.filepath = original_filepath
            if original_samples is not None:
                scene.cycles.samples = original_samples
                scene.cycles.use_denoising = True
            for obj in [photo_obj, *render_objects]:
                if obj.name in original_hide_render:
                    obj.hide_render = original_hide_render[obj.name]
            for obj in render_objects:
                obj.data.materials.clear()
                for mat in original_materials.get(obj.name, []):
                    if mat:
                        obj.data.materials.append(mat)

    clear_scene()
    calibration = load_json((ROOT / args.calibration).resolve())
    geo = geometry_loader.load(args.geometry + ".json")
    resolve_sun_position(args, calibration)
    resolve_sun_color(args)

    camera = setup_camera(calibration)
    photo_path = ROOT / calibration["image"]["path"]
    output_path = (ROOT / args.output).resolve()
    photo_obj = add_photo_plane(calibration, camera, photo_path, material_emission_image("Original photo", photo_path))
    setup_render(calibration)
    column_color = resolve_column_color(photo_path, photo_obj, output_path)

    materials = {
        "column": material_principled("Matched plaster columns", column_color, roughness=0.82),
        "wood": material_principled("Warm brown glulam", (0.42, 0.22, 0.09, 1), roughness=0.58),
        "rafter": material_principled("White painted rafters", (0.88, 0.86, 0.80, 1), roughness=0.64),
        "purlin": material_principled("White painted purlins", (0.88, 0.86, 0.80, 1), roughness=0.64),
        "solar": material_principled(
            "Bifacial glass laminate",
            (0.020, 0.032, 0.044, 1),
            roughness=0.18,
            alpha=1.0,
            transmission=0.0,
            specular=0.50,
            coat_weight=0.32,
            coat_roughness=0.13,
            ior=1.52,
        ),
        "solar_frame": material_principled(
            "Black anodized aluminium panel frames",
            (0.006, 0.006, 0.006, 1),
            roughness=0.56,
            metallic=0.35,
            alpha=1.0,
            specular=0.16,
            coat_weight=0.0,
        ),
        "solar_cell_grid": material_principled(
            "Subtle solar cell grid",
            (0.050, 0.064, 0.078, 1),
            roughness=0.48,
            alpha=1.0,
            specular=0.12,
            coat_weight=0.0,
        ),
        "solar_bottom_glow": material_principled_with_emission(
            "Bifacial underside cell glow",
            (0.018, 0.026, 0.032, 1),
            (0.30, 0.42, 0.50, 1),
            emission_strength=0.16,
            roughness=0.36,
            specular=0.10,
        ),
        "solar_bottom_cell_grid": material_principled_with_emission(
            "Bifacial underside cell pattern",
            (0.035, 0.050, 0.060, 1),
            (0.20, 0.30, 0.38, 1),
            emission_strength=0.09,
            roughness=0.42,
            specular=0.08,
        ),
        "glass": material_principled(
            "Clear terrace glass",
            (0.86, 0.96, 1.0, 0.14),
            roughness=0.012,
            alpha=0.14,
            transmission=0.78,
            specular=0.32,
            coat_weight=0.04,
            coat_roughness=0.02,
            ior=1.45,
        ),
        "boarding": material_principled("Brown boarding", (0.42, 0.27, 0.14, 1), roughness=0.72),
        "bounce_wall": material_emission_color(
            "Camera-invisible wall bounce",
            (
                args.bounce_wall_reflectance,
                args.bounce_wall_reflectance * 0.94,
                args.bounce_wall_reflectance * 0.84,
                1,
            ),
            strength=args.bounce_wall_energy,
        ),
        "bounce_floor": material_emission_color(
            "Camera-invisible floor bounce",
            (
                args.bounce_floor_reflectance,
                args.bounce_floor_reflectance * 0.92,
                args.bounce_floor_reflectance * 0.80,
                1,
            ),
            strength=args.bounce_floor_energy,
        ),
        "solar_sky_reflection": material_emission_color(
            "Camera-invisible sky reflection for solar glass",
            (0.42, 0.62, 0.95, 1),
            strength=args.solar_sky_reflection_energy,
        ),
    }
    render_color_materials: dict[tuple[float, float, float, float], object] = {}

    def material_with_render_color(default_mat, member: dict, roughness: float = 0.64):
        color = member_render_color(member)
        if color is None:
            return default_mat
        key = tuple(round(channel, 6) for channel in color)
        if key not in render_color_materials:
            render_color_materials[key] = material_principled(
                f"Geometry render color {rgb_to_hex(color)}",
                color,
                roughness=roughness,
            )
        return render_color_materials[key]

    render_objects = []

    member_index = build_member_index(geo)
    member_objects = {}
    for group in ("columns", "beams", "rafters", "purlins"):
        for member in geometry_loader.expanded_members(geo, group):
            if member.get("existing"):
                continue
            mat = materials["column"] if group == "columns" else materials["wood"]
            if group == "rafters":
                mat = materials["rafter"]
            if group == "purlins":
                mat = materials["purlin"]
            mat = material_with_render_color(mat, member)
            obj = create_box_member(member["id"], group, member, mat)
            if obj:
                member_index[member["id"]]["object"] = obj
                member_objects[member["id"]] = obj
                render_objects.append(obj)

    apply_notches(member_index, geo)

    for surface in geo.get("surfaces", []):
        if surface.get("existing"):
            continue
        poly = surface.get("polygon")
        if not poly:
            continue
        surface_type = surface.get("type")
        if surface_type == "solar_panel_array":
            render_objects.extend(create_solar_panel_array(surface, materials, camera))
        elif "glazing" in surface.get("id", "") or "glazing" in surface_type:
            obj = create_surface(surface["id"], poly, float(surface.get("thickness_mm", 0)), materials["glass"])
            if obj:
                render_objects.append(obj)
        elif surface_type == "boarding":
            obj = create_surface(surface["id"], poly, float(surface.get("thickness_mm", 0)), materials["boarding"])
            if obj:
                render_objects.append(obj)

    bounce_objects = create_bounce_surfaces(geo, render_objects, materials)
    sky_reflection_objects = create_solar_sky_reflection_surfaces(render_objects, materials)
    foreground_objects = create_foreground_occluders(material_emission_color("Foreground occluders", (1, 1, 1, 1), 1.0))

    setup_lighting(camera)
    setup_render(calibration)
    if args.save_blend:
        bpy.ops.wm.save_as_mainfile(filepath=str((ROOT / args.save_blend).resolve()))
    bpy.ops.render.render(write_still=True)
    print(f"Rendered {bpy.context.scene.render.filepath}")
    foreground_mask = None
    if foreground_objects:
        foreground_mask = render_foreground_mask(
            foreground_mask_path(output_path),
            photo_obj,
            render_objects,
            foreground_objects,
        )
        apply_foreground_occlusion_to_image(output_path, photo_path, foreground_mask)
    if args.ai_guides:
        render_ai_guides(calibration, camera, photo_obj, render_objects, output_path, foreground_mask)


if __name__ == "__main__":
    main()
