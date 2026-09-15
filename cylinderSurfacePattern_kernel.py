# -*- coding: utf-8 -*-
"""Abaqus/CAE kernel implementation for Cylinder Surface Pattern.

Supported by Abaqus/CAE 2020 (Python 2.7).  The source part is never edited;
all boolean and mesh work is performed on a generated output part.
"""
from __future__ import division

import math

from abaqus import mdb, session
from abaqusConstants import *
import mesh

from cylinderSurfacePattern_core import (
    PatternError, add, analyze_revolved_face, attachment_centers,
    attachment_counts_to_fill, axis_point_at, chocolate_groove_layout,
    cross, dot, matrix_axis_angle, profile_state, radial_at, scale,
    suggested_embed, surface_basis, surface_point_profile, tangent_at, unit)


PLUGIN_SET_NAME = 'CSP_ALL_CELLS'


def _single_picked_face(selection):
    """Accept Abaqus 2020 Face, FaceArray, tuple, or list pick results."""
    if selection is None:
        raise PatternError('Pick exactly one revolved exterior face.')
    try:
        count = len(selection)
    except TypeError:
        # AFXPickStep.ONE returns a bare Face in some Abaqus/CAE releases.
        face = selection
    else:
        if count != 1:
            raise PatternError('Pick exactly one revolved exterior face.')
        try:
            face = selection[0]
        except Exception:
            face = tuple(selection)[0]
    if not hasattr(face, 'index'):
        raise PatternError('The viewport selection is not a geometric Face.')
    return face


def _repository_has(repository, name):
    return name in repository.keys()


def _fresh_name(repository, root):
    if not _repository_has(repository, root):
        return root
    index = 1
    while _repository_has(repository, '%s_%d' % (root, index)):
        index += 1
    return '%s_%d' % (root, index)


def _instance_names_for_part(assembly, part_name):
    answer = []
    for name in assembly.instances.keys():
        instance = assembly.instances[name]
        if getattr(instance, 'partName', None) == part_name:
            answer.append(name)
    return answer


def _delete_output(model, output_name):
    assembly = model.rootAssembly
    for name in _instance_names_for_part(assembly, output_name):
        del assembly.instances[name]
    if _repository_has(model.parts, output_name):
        del model.parts[output_name]


def _create_box_part(model, name, width_x, width_y, depth_z):
    sheet = max(width_x, width_y, depth_z) * 10.0
    sketch_name = _fresh_name(model.sketches, '__CSP_BOX__')
    sketch = model.ConstrainedSketch(name=sketch_name, sheetSize=sheet)
    sketch.rectangle(point1=(-0.5 * width_x, -0.5 * width_y),
                     point2=(0.5 * width_x, 0.5 * width_y))
    part = model.Part(name=name, dimensionality=THREE_D,
                      type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=depth_z)
    del model.sketches[sketch_name]
    return part


def _create_sphere_part(model, name, radius):
    sheet = radius * 10.0
    sketch_name = _fresh_name(model.sketches, '__CSP_SPHERE__')
    sketch = model.ConstrainedSketch(name=sketch_name, sheetSize=sheet)
    sketch.ConstructionLine(point1=(0.0, -2.0 * radius),
                            point2=(0.0, 2.0 * radius))
    sketch.ArcByCenterEnds(center=(0.0, 0.0),
                           point1=(0.0, -radius),
                           point2=(0.0, radius),
                           direction=CLOCKWISE)
    sketch.Line(point1=(0.0, radius), point2=(0.0, -radius))
    part = model.Part(name=name, dimensionality=THREE_D,
                      type=DEFORMABLE_BODY)
    part.BaseSolidRevolve(sketch=sketch, angle=360.0)
    del model.sketches[sketch_name]
    return part


def _create_annular_sector_part(model, name, inner_radius, outer_radius,
                                angular_span, axial_length):
    """Create an exact cylindrical-band cutter along local +Z."""
    if inner_radius <= 0.0 or outer_radius <= inner_radius:
        raise PatternError('Invalid radial limits for a groove cutter.')
    if axial_length <= 0.0 or angular_span <= 0.0:
        raise PatternError('Invalid groove cutter length or angle.')
    sheet = max(outer_radius, axial_length) * 4.0
    sketch_name = _fresh_name(model.sketches, '__CSP_BAND__')
    sketch = model.ConstrainedSketch(name=sketch_name, sheetSize=sheet)
    full = angular_span >= 2.0 * math.pi - 1.0e-7
    if full:
        sketch.CircleByCenterPerimeter(
            center=(0.0, 0.0), point1=(outer_radius, 0.0))
        sketch.CircleByCenterPerimeter(
            center=(0.0, 0.0), point1=(inner_radius, 0.0))
    else:
        end_cos = math.cos(angular_span)
        end_sin = math.sin(angular_span)
        outer_start = (outer_radius, 0.0)
        outer_end = (outer_radius * end_cos, outer_radius * end_sin)
        inner_start = (inner_radius, 0.0)
        inner_end = (inner_radius * end_cos, inner_radius * end_sin)
        sketch.ArcByCenterEnds(
            center=(0.0, 0.0), point1=outer_start, point2=outer_end,
            direction=COUNTERCLOCKWISE)
        sketch.Line(point1=outer_end, point2=inner_end)
        sketch.ArcByCenterEnds(
            center=(0.0, 0.0), point1=inner_end, point2=inner_start,
            direction=CLOCKWISE)
        sketch.Line(point1=inner_start, point2=outer_start)
    part = model.Part(name=name, dimensionality=THREE_D,
                      type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=axial_length)
    del model.sketches[sketch_name]
    return part


def _create_revolved_profile_part(model, name, outer_points, inner_points,
                                  angular_span):
    """Create a cutter by revolving a closed meridional profile about +Y."""
    all_points = tuple(outer_points) + tuple(inner_points)
    if len(outer_points) < 2 or len(inner_points) < 2:
        raise PatternError('A revolved cutter needs two profile points.')
    if min(point[0] for point in all_points) <= 0.0:
        raise PatternError('Groove depth reaches or crosses the revolution '
                           'axis at part of the selected face.')
    extent = max(max(abs(point[0]), abs(point[1]))
                 for point in all_points)
    sketch_name = _fresh_name(model.sketches, '__CSP_REVOLVED_BAND__')
    sketch = model.ConstrainedSketch(
        name=sketch_name, sheetSize=max(1.0, 4.0 * extent))
    sketch.ConstructionLine(point1=(0.0, -2.0 * extent),
                            point2=(0.0, 2.0 * extent))
    if len(outer_points) == 2:
        sketch.Line(point1=outer_points[0], point2=outer_points[1])
    else:
        sketch.Spline(points=tuple(outer_points))
    sketch.Line(point1=outer_points[-1], point2=inner_points[-1])
    reversed_inner = tuple(reversed(inner_points))
    if len(reversed_inner) == 2:
        sketch.Line(point1=reversed_inner[0], point2=reversed_inner[1])
    else:
        sketch.Spline(points=reversed_inner)
    sketch.Line(point1=inner_points[0], point2=outer_points[0])
    part = model.Part(name=name, dimensionality=THREE_D,
                      type=DEFORMABLE_BODY)
    part.BaseSolidRevolve(
        sketch=sketch, angle=min(360.0, math.degrees(angular_span)))
    del model.sketches[sketch_name]
    return part


def _orient_and_place_box(assembly, instance_name, frame, theta,
                          meridional_coordinate, bottom_offset):
    x_axis, y_axis, z_axis = surface_basis(
        frame, theta, meridional_coordinate)
    rotation_axis, rotation_angle = matrix_axis_angle(
        x_axis, y_axis, z_axis)
    if abs(rotation_angle) > 1.0e-9:
        assembly.rotate(instanceList=(instance_name,),
                        axisPoint=(0.0, 0.0, 0.0),
                        axisDirection=rotation_axis,
                        angle=rotation_angle)
    origin = surface_point_profile(
        frame, theta, meridional_coordinate, bottom_offset)
    assembly.translate(instanceList=(instance_name,), vector=origin)


def _revolved_tool_sense(frame, angular_start):
    radial = radial_at(frame, angular_start)
    standard_tangent = unit(cross(frame['axis'], radial))
    return (1.0 if dot(tangent_at(frame, angular_start), standard_tangent)
            >= 0.0 else -1.0)


def _orient_and_place_revolved_tool(assembly, instance_name, frame,
                                    angular_start):
    """Map local revolve (X radial, Y axis) to the measured surface frame."""
    x_axis = radial_at(frame, angular_start)
    sense = _revolved_tool_sense(frame, angular_start)
    y_axis = scale(frame['axis'], sense)
    z_axis = unit(cross(x_axis, y_axis))
    rotation_axis, rotation_angle = matrix_axis_angle(
        x_axis, y_axis, z_axis)
    if abs(rotation_angle) > 1.0e-9:
        assembly.rotate(instanceList=(instance_name,),
                        axisPoint=(0.0, 0.0, 0.0),
                        axisDirection=rotation_axis,
                        angle=rotation_angle)
    assembly.translate(instanceList=(instance_name,),
                       vector=frame['axis_point'])


def _offset_band_points(frame, meridional_start, meridional_end,
                        inner_offset, outer_offset, angular_start,
                        sample_count=17):
    """Return a profile band between two local-normal surface offsets."""
    if outer_offset <= inner_offset:
        raise PatternError('Surface-layer outer offset must exceed its inner '
                           'offset.')
    sense = _revolved_tool_sense(frame, angular_start)
    count = max(2, int(sample_count))
    outer = []
    inner = []
    for index in range(count):
        ratio = index / float(count - 1)
        coordinate = (meridional_start +
                      ratio * (meridional_end - meridional_start))
        state = profile_state(frame, coordinate)
        outer.append((
            state['radius'] + outer_offset * state['normal_radius'],
            sense * (state['axial'] +
                     outer_offset * state['normal_axial'])))
        inner.append((
            state['radius'] + inner_offset * state['normal_radius'],
            sense * (state['axial'] +
                     inner_offset * state['normal_axial'])))
    return tuple(outer), tuple(inner)


def _revolved_band_points(frame, meridional_start, meridional_end,
                          groove_depth, overcut, angular_start,
                          sample_count=17):
    """Return outer/inner groove-cutter profiles."""
    return _offset_band_points(
        frame, meridional_start, meridional_end,
        -groove_depth, overcut, angular_start, sample_count)


def _orient_and_place_cylindrical_tool(
        assembly, instance_name, frame, angular_start, axial_start,
        axial_length):
    """Map local (radial, tangent, axis) to the measured cylinder frame."""
    x_axis = radial_at(frame, angular_start)
    y_axis = radial_at(frame, angular_start + 0.5 * math.pi)
    # A sector face can be parameterized clockwise or counterclockwise. Use
    # cross(X,Y) so the mapped basis is always a proper rotation; when local
    # +Z points opposite the measured cylinder axis, anchor at the other end.
    z_axis = unit(cross(x_axis, y_axis))
    rotation_axis, rotation_angle = matrix_axis_angle(
        x_axis, y_axis, z_axis)
    if abs(rotation_angle) > 1.0e-9:
        assembly.rotate(instanceList=(instance_name,),
                        axisPoint=(0.0, 0.0, 0.0),
                        axisDirection=rotation_axis,
                        angle=rotation_angle)
    axial_anchor = axial_start
    if dot(z_axis, frame['axis']) < 0.0:
        axial_anchor += axial_length
    assembly.translate(instanceList=(instance_name,),
                       vector=axis_point_at(frame, axial_anchor))


def _partition_axial(part, frame, coordinate):
    point_feature = part.DatumPointByCoordinate(
        coords=axis_point_at(frame, coordinate))
    plane_feature = part.DatumPlaneByPointNormal(
        point=part.datums[point_feature.id],
        normal=part.datums[frame['axis_datum_id']])
    part.PartitionCellByDatumPlane(cells=part.cells[:],
                                   datumPlane=part.datums[plane_feature.id])


def _partition_angular(part, frame, theta):
    radial = radial_at(frame, theta)
    t0 = frame['axial_min']
    t1 = frame['axial_max']
    tm = 0.5 * (t0 + t1)
    p0_feature = part.DatumPointByCoordinate(
        coords=axis_point_at(frame, t0))
    p1_feature = part.DatumPointByCoordinate(
        coords=axis_point_at(frame, t1))
    p2_feature = part.DatumPointByCoordinate(
        coords=add(axis_point_at(frame, tm),
                   scale(radial, frame['radius'])))
    plane_feature = part.DatumPlaneByThreePoints(
        point1=part.datums[p0_feature.id],
        point2=part.datums[p1_feature.id],
        point3=part.datums[p2_feature.id])
    part.PartitionCellByDatumPlane(cells=part.cells[:],
                                   datumPlane=part.datums[plane_feature.id])


def _partition_pattern_grid(part, frame, centers, rows, columns):
    """Partition the base copy between pattern rows and columns."""
    warnings = []
    if rows > 1:
        row_coordinates = []
        for row in range(rows):
            values = [item[1] for item in centers if item[2] == row]
            if values:
                row_coordinates.append(sum(values) / len(values))
        row_coordinates.sort()
        for index in range(len(row_coordinates) - 1):
            meridional = 0.5 * (row_coordinates[index] +
                                row_coordinates[index + 1])
            coordinate = profile_state(frame, meridional)['axial']
            try:
                _partition_axial(part, frame, coordinate)
            except Exception as exc:
                warnings.append('Axial partition at %.6g failed: %s' %
                                (coordinate, str(exc)))
    if columns > 1:
        first_row = [item for item in centers if item[2] == 0]
        first_row.sort(key=lambda item: item[3])
        used_planes = []
        for index in range(len(first_row) - 1):
            theta = 0.5 * (first_row[index][0] + first_row[index + 1][0])
            canonical = theta % math.pi
            duplicate = any(abs(canonical - old) < 1.0e-7 or
                            abs(abs(canonical - old) - math.pi) < 1.0e-7
                            for old in used_planes)
            if duplicate:
                continue
            used_planes.append(canonical)
            try:
                _partition_angular(part, frame, theta)
            except Exception as exc:
                warnings.append('Angular partition at %.6g rad failed: %s' %
                                (theta, str(exc)))
    return warnings


def _partition_chocolate_patch(part, frame, layout):
    """Create axial sweep layers without planes through the cylinder axis."""
    warnings = []
    axial_values = [layout['axial_start'], layout['axial_end']]
    half_width = 0.5 * layout['transverse_groove_width']
    for coordinate in layout['transverse_centers']:
        axial_values.extend((coordinate - half_width,
                             coordinate + half_width))
    axial_values.extend(sorted(set(
        item[1] for item in layout['block_centers'])))
    meridional_length = frame.get('meridional_length',
                                  frame['axial_length'])
    for meridional in axial_values:
        if (meridional <= 1.0e-7 or
                meridional >= meridional_length - 1.0e-7):
            continue
        coordinate = profile_state(frame, meridional)['axial']
        try:
            _partition_axial(part, frame, coordinate)
        except Exception as exc:
            warnings.append('Chocolate axial partition at %.6g failed: %s' %
                            (coordinate, str(exc)))

    # Do not add radial datum planes. Those planes contain the cylinder axis
    # and create tiny central mesh regions. Longitudinal groove side faces
    # already provide the required local outer-surface topology.
    return warnings


def _element_definition(mesh_type, library):
    if mesh_type == 0:
        return (TET, FREE, (mesh.ElemType(elemCode=C3D4,
                                         elemLibrary=library),))
    if mesh_type == 1:
        return (TET, FREE, (mesh.ElemType(elemCode=C3D10,
                                         elemLibrary=library),))
    if mesh_type == 2:
        return (TET, FREE, (mesh.ElemType(elemCode=C3D10M,
                                         elemLibrary=library),))
    if mesh_type == 3:
        return (HEX_DOMINATED, FREE, (
            mesh.ElemType(elemCode=C3D8R, elemLibrary=library),
            mesh.ElemType(elemCode=C3D6, elemLibrary=library),
            mesh.ElemType(elemCode=C3D4, elemLibrary=library)))
    if mesh_type == 4:
        return (HEX, SWEEP, (
            mesh.ElemType(elemCode=C3D8R, elemLibrary=library),
            mesh.ElemType(elemCode=C3D6, elemLibrary=library),
            mesh.ElemType(elemCode=C3D4, elemLibrary=library)))
    raise PatternError('Unknown mesh type index: %s' % mesh_type)


def _mesh_part(part, mesh_size, mesh_type, element_library):
    mesh_size = float(mesh_size)
    if mesh_size <= 0.0:
        raise PatternError('Mesh size must be positive.')
    if len(part.cells) == 0:
        raise PatternError('The generated output has no solid cells.')
    library = STANDARD if int(element_library) == 0 else EXPLICIT
    shape, technique, element_types = _element_definition(
        int(mesh_type), library)
    cells = part.cells[:]
    try:
        part.setMeshControls(regions=cells, elemShape=shape,
                             technique=technique)
    except Exception as exc:
        if int(mesh_type) in (3, 4):
            raise PatternError(
                'The requested hex/hex-dominated mesh control is not '
                'available for this boolean topology. Use C3D4, C3D10, or '
                'C3D10M free tetrahedral mesh. Abaqus message: %s' % str(exc))
        raise
    part.setElementType(regions=(cells,), elemTypes=element_types)
    part.seedPart(size=mesh_size, deviationFactor=0.1,
                  minSizeFactor=0.1)
    try:
        part.generateMesh()
    except Exception as exc:
        raise PatternError(
            'Mesh generation failed for the requested mesh type. Boolean '
            'grooves and sphere/cube intersections are normally most robust '
            'with a free tetrahedral type (C3D4/C3D10/C3D10M). Abaqus '
            'message: %s' % str(exc))
    if len(part.elements) == 0:
        raise PatternError('Abaqus generated zero elements.')
    if _repository_has(part.sets, PLUGIN_SET_NAME):
        del part.sets[PLUGIN_SET_NAME]
    part.Set(name=PLUGIN_SET_NAME, cells=part.cells[:])


def _mesh_part_with_hex_fallback(part, mesh_size, mesh_type,
                                 element_library):
    """Retry any failed hex request with free C3D4 without losing geometry."""
    requested = int(mesh_type)
    try:
        _mesh_part(part, mesh_size, requested, element_library)
        return requested, None
    except PatternError as hex_error:
        if requested not in (3, 4):
            raise
        try:
            if len(part.elements):
                part.deleteMesh(regions=part.cells[:])
        except Exception:
            try:
                part.deleteMesh()
            except Exception:
                pass
        try:
            _mesh_part(part, mesh_size, 0, element_library)
        except Exception as tet_error:
            raise PatternError(
                'Hex meshing failed and the automatic C3D4 retry also '
                'failed. Hex message: %s; C3D4 message: %s' %
                (str(hex_error), str(tet_error)))
        warning = (
            'Abaqus could not sweep this topology, so the failed hex mesh '
            'was cleared and automatically regenerated as free C3D4. '
            'Original message: %s' % str(hex_error))
        return 0, warning


def _show_result(part):
    try:
        viewport = session.viewports[session.currentViewportName]
        viewport.setValues(displayedObject=part)
    except Exception:
        pass


def _groove_result_revolved(model, temp_part, frame, output_name, layout,
                            groove_depth, mesh_size):
    """Cut profile-following bands from a variable-radius revolved face."""
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    base_instance_name = _fresh_name(assembly.instances, '__CSP_BASE__')
    base_instance = assembly.Instance(name=base_instance_name,
                                      part=temp_part, dependent=ON)
    overcut = max(0.1 * groove_depth, 0.25 * mesh_size,
                  frame['radius'] * 1.0e-4)
    cutter_instances = []
    cutter_part_names = []
    temporary_instance_names = [base_instance_name]
    try:
        half_width = 0.5 * layout['transverse_groove_width']
        for index, meridional in enumerate(layout['transverse_centers']):
            angular_start = layout['angular_start']
            outer, inner = _revolved_band_points(
                frame, meridional - half_width, meridional + half_width,
                groove_depth, overcut, angular_start, sample_count=5)
            part_name = _fresh_name(
                model.parts, '__CSP_CROSS_PROFILE_%04d__' % index)
            cutter_part = _create_revolved_profile_part(
                model, part_name, outer, inner, layout['angular_span'])
            cutter_part_names.append(part_name)
            instance_name = _fresh_name(
                assembly.instances, '__CSP_CROSS_%04d__' % index)
            instance = assembly.Instance(
                name=instance_name, part=cutter_part, dependent=ON)
            temporary_instance_names.append(instance_name)
            _orient_and_place_revolved_tool(
                assembly, instance_name, frame, angular_start)
            cutter_instances.append(instance)

        for index, theta in enumerate(layout['longitudinal_centers']):
            angular_start = (theta -
                             0.5 * layout['longitudinal_groove_angle'])
            outer, inner = _revolved_band_points(
                frame, layout['meridional_start'],
                layout['meridional_end'], groove_depth, overcut,
                angular_start, sample_count=33)
            part_name = _fresh_name(
                model.parts, '__CSP_LONG_PROFILE_%04d__' % index)
            cutter_part = _create_revolved_profile_part(
                model, part_name, outer, inner,
                layout['longitudinal_groove_angle'])
            cutter_part_names.append(part_name)
            instance_name = _fresh_name(
                assembly.instances, '__CSP_LONG_%04d__' % index)
            instance = assembly.Instance(
                name=instance_name, part=cutter_part, dependent=ON)
            temporary_instance_names.append(instance_name)
            _orient_and_place_revolved_tool(
                assembly, instance_name, frame, angular_start)
            cutter_instances.append(instance)

        if not cutter_instances:
            raise PatternError('The requested block layout creates no '
                               'crossed grooves.')
        assembly.regenerate()
        result_instance = assembly.InstanceFromBooleanCut(
            name=output_name, instanceToBeCut=base_instance,
            cuttingInstances=tuple(cutter_instances),
            originalInstances=DELETE)
        assembly.regenerate()
        return model.parts[output_name], result_instance
    finally:
        for name in temporary_instance_names:
            if _repository_has(assembly.instances, name):
                del assembly.instances[name]
        for part_name in cutter_part_names:
            if (_repository_has(model.parts, part_name) and
                    not _instance_names_for_part(assembly, part_name)):
                del model.parts[part_name]


def _groove_result(model, temp_part, frame, output_name, layout,
                   transverse_groove_width, longitudinal_groove_width,
                   groove_depth, mesh_size):
    if frame.get('surface_kind', 'cylinder') != 'cylinder':
        return _groove_result_revolved(
            model, temp_part, frame, output_name, layout,
            groove_depth, mesh_size)
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    base_instance_name = _fresh_name(assembly.instances, '__CSP_BASE__')
    base_instance = assembly.Instance(name=base_instance_name,
                                      part=temp_part, dependent=ON)
    overcut = max(0.1 * groove_depth, 0.25 * mesh_size,
                  frame['radius'] * 1.0e-4)
    inner_radius = frame['radius'] - groove_depth
    outer_radius = frame['radius'] + overcut
    cutter_parts = []
    transverse_part = None
    longitudinal_part = None
    if layout['transverse_centers']:
        name = _fresh_name(model.parts, '__CSP_TRANSVERSE_GROOVE__')
        transverse_part = _create_annular_sector_part(
            model, name, inner_radius, outer_radius,
            layout['angular_span'], transverse_groove_width)
        cutter_parts.append((name, transverse_part))
    if layout['longitudinal_centers']:
        name = _fresh_name(model.parts, '__CSP_LONGITUDINAL_GROOVE__')
        longitudinal_part = _create_annular_sector_part(
            model, name, inner_radius, outer_radius,
            layout['longitudinal_groove_angle'], layout['axial_length'])
        cutter_parts.append((name, longitudinal_part))
    cutter_instances = []
    temporary_instance_names = [base_instance_name]
    try:
        for index, axial in enumerate(layout['transverse_centers']):
            name = _fresh_name(
                assembly.instances, '__CSP_CROSS_%04d__' % index)
            instance = assembly.Instance(name=name, part=transverse_part,
                                         dependent=ON)
            temporary_instance_names.append(name)
            _orient_and_place_cylindrical_tool(
                assembly, name, frame, layout['angular_start'],
                axial - 0.5 * transverse_groove_width,
                transverse_groove_width)
            cutter_instances.append(instance)
        for index, theta in enumerate(layout['longitudinal_centers']):
            name = _fresh_name(
                assembly.instances, '__CSP_LONG_%04d__' % index)
            instance = assembly.Instance(name=name, part=longitudinal_part,
                                         dependent=ON)
            temporary_instance_names.append(name)
            _orient_and_place_cylindrical_tool(
                assembly, name, frame,
                theta - 0.5 * layout['longitudinal_groove_angle'],
                layout['axial_start'], layout['axial_length'])
            cutter_instances.append(instance)
        if not cutter_instances:
            raise PatternError('The requested block layout creates no '
                               'crossed grooves.')
        assembly.regenerate()
        result_instance = assembly.InstanceFromBooleanCut(
            name=output_name, instanceToBeCut=base_instance,
            cuttingInstances=tuple(cutter_instances),
            originalInstances=DELETE)
        assembly.regenerate()
        return model.parts[output_name], result_instance
    finally:
        for name in temporary_instance_names:
            if _repository_has(assembly.instances, name):
                del assembly.instances[name]
        for part_name, unused_part in cutter_parts:
            if (_repository_has(model.parts, part_name) and
                    not _instance_names_for_part(assembly, part_name)):
                del model.parts[part_name]


def _create_offset_band_instance(
        model, assembly, frame, part_root, instance_root,
        meridional_start, meridional_end, angular_start, angular_span,
        inner_offset, outer_offset, sample_count=33):
    outer, inner = _offset_band_points(
        frame, meridional_start, meridional_end,
        inner_offset, outer_offset, angular_start, sample_count)
    part_name = _fresh_name(model.parts, part_root)
    band_part = _create_revolved_profile_part(
        model, part_name, outer, inner, angular_span)
    instance_name = _fresh_name(assembly.instances, instance_root)
    instance = assembly.Instance(
        name=instance_name, part=band_part, dependent=ON)
    _orient_and_place_revolved_tool(
        assembly, instance_name, frame, angular_start)
    return part_name, instance_name, instance


def _create_offset_groove_cutters(
        model, assembly, frame, layout, inner_offset, outer_offset):
    part_names = []
    instance_names = []
    instances = []
    half_width = 0.5 * layout['transverse_groove_width']
    for index, meridional in enumerate(layout['transverse_centers']):
        part_name, instance_name, instance = _create_offset_band_instance(
            model, assembly, frame,
            '__CSP_LAYER_CROSS_%04d__' % index,
            '__CSP_LAYER_CROSS_INST_%04d__' % index,
            meridional - half_width, meridional + half_width,
            layout['angular_start'], layout['angular_span'],
            inner_offset, outer_offset, sample_count=5)
        part_names.append(part_name)
        instance_names.append(instance_name)
        instances.append(instance)
    for index, theta in enumerate(layout['longitudinal_centers']):
        angular_start = (theta -
                         0.5 * layout['longitudinal_groove_angle'])
        part_name, instance_name, instance = _create_offset_band_instance(
            model, assembly, frame,
            '__CSP_LAYER_LONG_%04d__' % index,
            '__CSP_LAYER_LONG_INST_%04d__' % index,
            layout['meridional_start'], layout['meridional_end'],
            angular_start, layout['longitudinal_groove_angle'],
            inner_offset, outer_offset, sample_count=33)
        part_names.append(part_name)
        instance_names.append(instance_name)
        instances.append(instance)
    return part_names, instance_names, instances


def _groove_result_layered(
        model, temp_part, frame, output_name, layout,
        groove_depth, raised_thickness, liner_thickness,
        construction_mode, mesh_size):
    """Create a parallel liner and/or raised grooved surface layer.

    mode 0 cuts from the liner outer surface into the merged base. Mode 1
    grooves only an added outer layer, leaving the copied source body intact.
    """
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    created_part_names = []
    created_instance_names = []
    base_name = _fresh_name(assembly.instances, '__CSP_BASE__')
    base_instance = assembly.Instance(
        name=base_name, part=temp_part, dependent=ON)
    created_instance_names.append(base_name)
    meridional_length = frame.get('meridional_length',
                                  frame['axial_length'])
    overcut = max(0.1 * max(groove_depth, raised_thickness, 1.0e-6),
                  0.25 * mesh_size, frame['radius'] * 1.0e-4)
    interface_overlap = max(frame['radius'] * 1.0e-6,
                            min(0.02 * max(raised_thickness,
                                           liner_thickness, 1.0),
                                0.02 * mesh_size))
    liner_instance = None
    try:
        if liner_thickness > 0.0:
            part_name, instance_name, liner_instance = (
                _create_offset_band_instance(
                    model, assembly, frame, '__CSP_PARALLEL_LINER__',
                    '__CSP_PARALLEL_LINER_INST__',
                    0.0, meridional_length, 0.0,
                    frame['angular_span'], -interface_overlap,
                    liner_thickness, sample_count=49))
            created_part_names.append(part_name)
            created_instance_names.append(instance_name)

        if int(construction_mode) == 0:
            target_instance = base_instance
            if liner_instance is not None:
                pre_name = _fresh_name(model.parts, '__CSP_PRE_GROOVE__')
                target_instance = assembly.InstanceFromBooleanMerge(
                    name=pre_name,
                    instances=(base_instance, liner_instance),
                    keepIntersections=OFF, originalInstances=DELETE,
                    domain=GEOMETRY)
                created_part_names.append(pre_name)
                created_instance_names.append(target_instance.name)
            cutter_parts, cutter_names, cutters = (
                _create_offset_groove_cutters(
                    model, assembly, frame, layout,
                    liner_thickness - groove_depth,
                    liner_thickness + overcut))
            created_part_names.extend(cutter_parts)
            created_instance_names.extend(cutter_names)
            result_instance = assembly.InstanceFromBooleanCut(
                name=output_name, instanceToBeCut=target_instance,
                cuttingInstances=tuple(cutters), originalInstances=DELETE)
            assembly.regenerate()
            return model.parts[output_name], result_instance

        # Raised mode: pattern only the added layer, then merge the surviving
        # blocks with the untouched base and optional continuous liner.
        raised_part, raised_name, raised_instance = (
            _create_offset_band_instance(
                model, assembly, frame, '__CSP_RAISED_LAYER__',
                '__CSP_RAISED_LAYER_INST__',
                layout['meridional_start'], layout['meridional_end'],
                layout['angular_start'], layout['angular_span'],
                liner_thickness - interface_overlap,
                liner_thickness + raised_thickness, sample_count=49))
        created_part_names.append(raised_part)
        created_instance_names.append(raised_name)
        cutter_parts, cutter_names, cutters = (
            _create_offset_groove_cutters(
                model, assembly, frame, layout,
                liner_thickness - overcut,
                liner_thickness + raised_thickness + overcut))
        created_part_names.extend(cutter_parts)
        created_instance_names.extend(cutter_names)
        patterned_name = _fresh_name(model.parts, '__CSP_RAISED_PATTERN__')
        patterned_instance = assembly.InstanceFromBooleanCut(
            name=patterned_name, instanceToBeCut=raised_instance,
            cuttingInstances=tuple(cutters), originalInstances=DELETE)
        created_part_names.append(patterned_name)
        created_instance_names.append(patterned_instance.name)
        merge_instances = [base_instance, patterned_instance]
        if liner_instance is not None:
            merge_instances.insert(1, liner_instance)
        result_instance = assembly.InstanceFromBooleanMerge(
            name=output_name, instances=tuple(merge_instances),
            keepIntersections=OFF, originalInstances=DELETE,
            domain=GEOMETRY)
        assembly.regenerate()
        return model.parts[output_name], result_instance
    finally:
        for name in created_instance_names:
            if _repository_has(assembly.instances, name):
                del assembly.instances[name]
        for part_name in created_part_names:
            if (part_name != output_name and
                    _repository_has(model.parts, part_name) and
                    not _instance_names_for_part(assembly, part_name)):
                del model.parts[part_name]


def _attachment_result(model, temp_part, frame, output_name, centers,
                       body_kind, body_size, signed_embed,
                       meridional_size=None, circumferential_size=None):
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    base_instance_name = _fresh_name(assembly.instances, '__CSP_BASE__')
    base_instance = assembly.Instance(name=base_instance_name,
                                      part=temp_part, dependent=ON)
    tool_name = _fresh_name(model.parts, '__CSP_ATTACHMENT__')
    if body_kind == 'sphere':
        attachment_part = _create_sphere_part(
            model, tool_name, 0.5 * body_size)
    else:
        cube_meridional = (body_size if meridional_size is None or
                           float(meridional_size) <= 0.0 else
                           float(meridional_size))
        cube_circumferential = (body_size
                                if circumferential_size is None or
                                float(circumferential_size) <= 0.0 else
                                float(circumferential_size))
        attachment_part = _create_box_part(
            model, tool_name, cube_circumferential,
            cube_meridional, body_size)
    instances = [base_instance]
    temporary_instance_names = [base_instance_name]
    try:
        for index, center in enumerate(centers):
            theta, meridional = center[0], center[1]
            layer_normal_offset = center[5] if len(center) > 5 else 0.0
            name = _fresh_name(
                assembly.instances, '__CSP_ATTACH_%04d__' % index)
            instance = assembly.Instance(name=name, part=attachment_part,
                                         dependent=ON)
            temporary_instance_names.append(name)
            if body_kind == 'sphere':
                center_point = surface_point_profile(
                    frame, theta, meridional,
                    0.5 * body_size - signed_embed + layer_normal_offset)
                assembly.translate(instanceList=(name,), vector=center_point)
            else:
                _orient_and_place_box(
                    assembly, name, frame, theta, meridional,
                    -signed_embed + layer_normal_offset)
            instances.append(instance)
        assembly.regenerate()
        result_instance = assembly.InstanceFromBooleanMerge(
            name=output_name, instances=tuple(instances),
            keepIntersections=OFF, originalInstances=DELETE, domain=GEOMETRY)
        assembly.regenerate()
        return model.parts[output_name], result_instance
    finally:
        for name in temporary_instance_names:
            if _repository_has(assembly.instances, name):
                del assembly.instances[name]
        if (_repository_has(model.parts, tool_name) and
                not _instance_names_for_part(assembly, tool_name)):
            del model.parts[tool_name]


def createCylinderSurfacePattern(
        modelName='Model-1', sourcePartName='Part-1',
        outputPartName='CylinderSurfacePattern', operation=0,
        grooveRows=5, grooveColumns=4, grooveAxialLength=0.8,
        grooveCircumWidth=0.8, grooveDepth=1.0,
        grooveBlockAxialLength=2.0, grooveBlockCircumWidth=2.0,
        grooveAxialOffset=0.0, grooveAngularOffset=0.0,
        fitGroovePatchToFace=True, grooveEndMargin=1.0,
        attachmentRows=5, attachmentColumns=4, attachmentSize=1.0,
        packing=0, embedFraction=0.05, fitAttachmentsToFace=False,
        detachedAttachments=False, attachmentClearance=0.1,
        meshSize=1.0, meshType=4, elementLibrary=0,
        autoPartition=True, overwrite=False, faces=None,
        grooveConstructionMode=0, grooveRaisedThickness=1.0,
        grooveLinerThickness=0.0, attachmentMeridianSize=0.0,
        attachmentCircumSize=0.0, attachmentRowShiftDegrees=10.0,
        attachmentLayers=1, attachmentLayerPacking=0):
    """Create grooves, cubes, or spheres on one picked revolved face.

    operation: 0=grooves, 1=cubes, 2=spheres. Defaults intentionally mirror
    the GUI so a CAE session holding an older form cannot fail with a raw
    argument-count TypeError; all parameters added after the original 22 use
    safe defaults so forms left in an older CAE session remain compatible.
    """
    if modelName not in mdb.models.keys():
        raise PatternError('Model "%s" does not exist.' % modelName)
    model = mdb.models[modelName]
    if sourcePartName not in model.parts.keys():
        raise PatternError('Part "%s" does not exist in model "%s".' %
                           (sourcePartName, modelName))
    if not outputPartName or outputPartName.strip() == '':
        raise PatternError('Output part name cannot be empty.')
    outputPartName = outputPartName.strip()
    picked_face = _single_picked_face(faces)
    operation = int(operation)
    if operation not in (0, 1, 2):
        raise PatternError('Operation must be grooves, cubes, or spheres.')
    construction_mode = int(grooveConstructionMode)
    liner_thickness = float(grooveLinerThickness)
    raised_thickness = float(grooveRaisedThickness)
    if construction_mode not in (0, 1):
        raise PatternError('Groove construction must cut the source or add '
                           'a raised grooved layer.')
    if liner_thickness < 0.0:
        raise PatternError('Parallel outer liner thickness cannot be '
                           'negative.')
    if operation == 0:
        if construction_mode == 0 and float(grooveDepth) <= 0.0:
            raise PatternError('Groove depth must be positive.')
        if construction_mode == 1 and raised_thickness <= 0.0:
            raise PatternError('Raised grooved-block thickness must be '
                               'positive.')

    source_part = model.parts[sourcePartName]
    if outputPartName == sourcePartName:
        raise PatternError('Output part name must differ from the source part '
                           'name. The source part is always preserved.')
    face_index = int(picked_face.index)
    if face_index < 0 or face_index >= len(source_part.faces):
        raise PatternError('The picked face does not belong to the named '
                           'source part. Pick in Part context and verify the '
                           'model/part fields.')
    try:
        same_face = picked_face == source_part.faces[face_index]
    except Exception:
        same_face = True
    if not same_face:
        raise PatternError('The picked face belongs to a different part. '
                           'Pick a face from the part named in Source part.')
    if _repository_has(model.parts, outputPartName):
        if not bool(overwrite):
            raise PatternError('Output part "%s" already exists. Choose a '
                               'new name or enable overwrite.' % outputPartName)
        _delete_output(model, outputPartName)

    temp_name = _fresh_name(model.parts, '__CSP_SOURCE_COPY__')
    temp_part = model.Part(name=temp_name, objectToCopy=source_part)
    result_part = None
    partition_warnings = []
    try:
        temp_face = temp_part.faces[face_index]
        frame = analyze_revolved_face(temp_part, temp_face)
        if operation == 0:
            if (construction_mode == 0 and
                    float(grooveDepth) - liner_thickness >=
                    frame['minimum_radius']):
                raise PatternError('Groove depth must be smaller than the '
                                   'liner thickness plus the smallest radius '
                                   'on the picked face.')
            layout = chocolate_groove_layout(
                frame, grooveRows, grooveColumns,
                grooveBlockAxialLength, grooveBlockCircumWidth,
                grooveAxialLength, grooveCircumWidth,
                grooveAxialOffset, grooveAngularOffset,
                fitGroovePatchToFace, grooveEndMargin)
            if bool(autoPartition) and construction_mode == 0:
                partition_warnings = _partition_chocolate_patch(
                    temp_part, frame, layout)
            if construction_mode == 1 or liner_thickness > 0.0:
                result_part, result_instance = _groove_result_layered(
                    model, temp_part, frame, outputPartName, layout,
                    float(grooveDepth), raised_thickness,
                    liner_thickness, construction_mode, float(meshSize))
            else:
                result_part, result_instance = _groove_result(
                    model, temp_part, frame, outputPartName, layout,
                    float(grooveAxialLength), float(grooveCircumWidth),
                    float(grooveDepth), float(meshSize))
            centers = layout['block_centers']
            if construction_mode == 1:
                pattern_kind = 'raised chocolate grooved blocks'
            elif liner_thickness > 0.0:
                pattern_kind = 'lined chocolate crossed grooves'
            else:
                pattern_kind = 'chocolate crossed grooves'
        else:
            body_kind = 'cube' if operation == 1 else 'sphere'
            detached = bool(detachedAttachments)
            if detached:
                clearance = float(attachmentClearance)
                if clearance <= 0.0:
                    raise PatternError('Detached surface clearance must be '
                                       'positive so bodies do not touch.')
                signed_embed = -clearance
            else:
                clearance = 0.0
                footprint_size = float(attachmentSize)
                if body_kind == 'cube':
                    footprint_size = max(
                        footprint_size,
                        (float(attachmentMeridianSize)
                         if float(attachmentMeridianSize) > 0.0 else
                         float(attachmentSize)),
                        (float(attachmentCircumSize)
                         if float(attachmentCircumSize) > 0.0 else
                         float(attachmentSize)))
                signed_embed = suggested_embed(
                    float(attachmentSize), frame['radius'], body_kind,
                    float(embedFraction), footprint_size)
            if bool(fitAttachmentsToFace):
                used_rows, used_columns = attachment_counts_to_fill(
                    frame, attachmentSize, packing, body_kind, signed_embed,
                    attachmentMeridianSize, attachmentCircumSize)
            else:
                used_rows = int(attachmentRows)
                used_columns = int(attachmentColumns)
            centers = attachment_centers(
                frame, used_rows, used_columns,
                attachmentSize, packing, body_kind, signed_embed,
                attachmentMeridianSize, attachmentCircumSize,
                attachmentRowShiftDegrees, attachmentLayers,
                attachmentLayerPacking)
            # Do not partition the revolved base body for attachments. The
            # merge also removes internal intersection boundaries, producing
            # a clean union for embedded bodies and disconnected cells for a
            # positive-clearance cover.
            result_part, result_instance = _attachment_result(
                model, temp_part, frame, outputPartName, centers,
                body_kind, float(attachmentSize), signed_embed,
                attachmentMeridianSize, attachmentCircumSize)
            pattern_kind = ('detached %s surface cover' % body_kind
                            if detached else '%s attachments' % body_kind)

        requested_mesh_type = int(meshType)
        effective_mesh_type = requested_mesh_type
        mesh_fallback = None
        if requested_mesh_type in (3, 4) and (
                operation in (1, 2) or
                frame.get('surface_kind') != 'cylinder' or
                construction_mode == 1 or liner_thickness > 0.0):
            effective_mesh_type = 0
            mesh_fallback = (
                'The requested hexahedral mesh was changed to free C3D4. '
                'Merged attachments and variable-radius revolved groove '
                'topologies are not reliably sweepable in Abaqus.')
        effective_mesh_type, runtime_fallback = _mesh_part_with_hex_fallback(
            result_part, meshSize, effective_mesh_type, elementLibrary)
        if runtime_fallback:
            mesh_fallback = runtime_fallback
        model.rootAssembly.regenerate()
        _show_result(result_part)
        for warning in partition_warnings:
            print('CylinderSurfacePattern warning: %s' % warning)
        if mesh_fallback:
            print('CylinderSurfacePattern warning: %s' % mesh_fallback)
        message = ('CylinderSurfacePattern created %s: part=%s, count=%d, '
                   'cells=%d, elements=%d, nodes=%d' %
                   (pattern_kind, outputPartName, len(centers),
                    len(result_part.cells), len(result_part.elements),
                    len(result_part.nodes)))
        print(message)
        return {
            'partName': outputPartName,
            'operation': pattern_kind,
            'count': len(centers),
            'cells': len(result_part.cells),
            'elements': len(result_part.elements),
            'nodes': len(result_part.nodes),
            'surfaceKind': frame.get('surface_kind', 'cylinder'),
            'meshTypeRequested': requested_mesh_type,
            'meshTypeUsed': effective_mesh_type,
            'meshFallback': mesh_fallback,
            'attachmentRowsUsed': (used_rows if operation in (1, 2)
                                   else None),
            'attachmentColumnsUsed': (used_columns if operation in (1, 2)
                                      else None),
            'detachedAttachments': (detached if operation in (1, 2)
                                    else False),
            'attachmentClearance': (clearance if operation in (1, 2)
                                    else 0.0),
            'attachmentLayersUsed': (int(attachmentLayers)
                                     if operation in (1, 2) else 0),
            'grooveConstructionMode': (construction_mode
                                       if operation == 0 else None),
            'grooveLinerThickness': (liner_thickness
                                     if operation == 0 else 0.0),
            'partitionWarnings': tuple(partition_warnings),
        }
    except PatternError:
        if _repository_has(model.parts, outputPartName):
            _delete_output(model, outputPartName)
        raise
    except Exception as exc:
        if _repository_has(model.parts, outputPartName):
            _delete_output(model, outputPartName)
        raise PatternError('Abaqus could not create the surface pattern: %s' %
                           str(exc))
    finally:
        if _repository_has(model.parts, temp_name):
            # Boolean operations delete the temporary instance first. If an
            # early error leaves one behind, remove only instances of this
            # plugin-owned temporary part.
            for instance_name in _instance_names_for_part(
                    model.rootAssembly, temp_name):
                del model.rootAssembly.instances[instance_name]
            del model.parts[temp_name]
