# -*- coding: utf-8 -*-
"""Geometry-independent helpers for Cylinder Surface Pattern.

This module deliberately uses Python 2.7-compatible syntax because Abaqus
2020 embeds Python 2.7.  It can also be imported by a normal Python 3 test.
"""
from __future__ import division

import math


TOL = 1.0e-9


class PatternError(ValueError):
    pass


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, value):
    return (a[0] * value, a[1] * value, a[2] * value)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a, label='vector'):
    length = norm(a)
    if length <= TOL:
        raise PatternError('%s has zero length.' % label)
    return scale(a, 1.0 / length)


def clamp(value, low, high):
    return max(low, min(high, value))


def rotate_in_plane(u, v, angle):
    return add(scale(u, math.cos(angle)), scale(v, math.sin(angle)))


def radial_component(point, axis_point, axis):
    relative = sub(point, axis_point)
    return sub(relative, scale(axis, dot(relative, axis)))


def axis_coordinate(point, axis_point, axis):
    return dot(sub(point, axis_point), axis)


def _point_on(entity):
    value = entity.pointOn
    if value and isinstance(value[0], (tuple, list)):
        value = value[0]
    return (float(value[0]), float(value[1]), float(value[2]))


def _distance(a, b):
    return norm(sub(a, b))


def _circle_from_edge(edge):
    """Return (center, radius, angular span) for a circular edge or None."""
    centers = []
    radii = []
    for parameter in (0.0, 0.25, 0.5, 0.75):
        try:
            data = edge.getCurvature(parameter=parameter)
            radius = float(data['radius'])
            curvature = tuple(data['curvature'])
            point = tuple(data['evaluationPoint'])
        except Exception:
            return None
        curvature_squared = dot(curvature, curvature)
        if (radius <= TOL or radius > 1.0e10 or
                curvature_squared <= 1.0e-20):
            return None
        centers.append(add(point, scale(curvature,
                                        1.0 / curvature_squared)))
        radii.append(radius)
    center = scale((sum(item[0] for item in centers),
                    sum(item[1] for item in centers),
                    sum(item[2] for item in centers)), 1.0 / len(centers))
    radius = sum(radii) / len(radii)
    tolerance = max(1.0e-6, radius * 1.0e-5)
    if max(_distance(item, center) for item in centers) > tolerance:
        return None
    if max(abs(item - radius) for item in radii) > tolerance:
        return None
    angular_span = float(edge.getSize(printResults=False)) / radius
    if angular_span > 2.0 * math.pi + 1.0e-4:
        return None
    return center, radius, clamp(angular_span, 1.0e-8, 2.0 * math.pi)


def _sample_meridian_edge(part, edge, sample_count=33):
    points = []
    for index in range(sample_count):
        parameter = index / float(sample_count - 1)
        try:
            data = edge.getCurvature(parameter=parameter)
            point = tuple(data['evaluationPoint'])
            points.append(point)
        except Exception:
            pass
    if len(points) < 2:
        for vertex_index in edge.getVertices():
            points.append(_point_on(part.vertices[vertex_index]))
        points.append(_point_on(edge))
    answer = []
    for point in points:
        if not answer or _distance(point, answer[-1]) > 1.0e-8:
            answer.append(point)
    return answer


def _unique_directions(directions, cosine_tolerance=1.0e-6):
    answer = []
    for direction in directions:
        duplicate = False
        for old in answer:
            if dot(direction, old) > 1.0 - cosine_tolerance:
                duplicate = True
                break
        if not duplicate:
            answer.append(direction)
    return answer


def _sector_basis(axis, axis_point, face_point, boundary_directions,
                  angular_span):
    """Return radial vectors at the lower angular boundary.

    Imported CAD can reverse an edge or the cylinder axis.  Test both senses
    and choose the interval that has the measured area and contains pointOn.
    """
    face_radial = unit(radial_component(face_point, axis_point, axis),
                       'cylindrical face radial direction')
    full = angular_span >= 2.0 * math.pi - 1.0e-5
    if full:
        return face_radial, unit(cross(axis, face_radial)), 2.0 * math.pi

    candidates = _unique_directions(boundary_directions)
    best = None
    for start in candidates:
        base_tangent = unit(cross(axis, start))
        for sense in (1.0, -1.0):
            tangent = scale(base_tangent, sense)
            face_angle = math.atan2(dot(face_radial, tangent),
                                    dot(face_radial, start))
            if face_angle < 0.0:
                face_angle += 2.0 * math.pi
            for finish in candidates:
                if finish is start:
                    continue
                finish_angle = math.atan2(dot(finish, tangent),
                                          dot(finish, start))
                if finish_angle < 0.0:
                    finish_angle += 2.0 * math.pi
                error = abs(finish_angle - angular_span)
                outside = 0.0
                if face_angle > angular_span + 1.0e-5:
                    outside = 10.0 + face_angle - angular_span
                score = error + outside
                if best is None or score < best[0]:
                    best = (score, start, tangent)
    if best is not None and best[0] < max(1.0e-3, angular_span * 1.0e-3):
        return best[1], best[2], angular_span

    # A fallback for unusual imported faces whose boundary has been split.
    tangent = unit(cross(axis, face_radial))
    start = unit(rotate_in_plane(
        face_radial, tangent, -0.5 * angular_span))
    return start, unit(cross(axis, start)), angular_span


def _profile_from_samples(samples, axis_point, axis):
    """Convert monotonic meridian samples into (arc length, axial, radius)."""
    values = []
    for point in samples:
        axial = axis_coordinate(point, axis_point, axis)
        radius = norm(radial_component(point, axis_point, axis))
        if radius > TOL:
            values.append((axial, radius))
    values.sort(key=lambda item: item[0])
    compact = []
    for axial, radius in values:
        if compact and abs(axial - compact[-1][0]) <= 1.0e-7:
            compact[-1] = (0.5 * (compact[-1][0] + axial),
                           0.5 * (compact[-1][1] + radius))
        else:
            compact.append((axial, radius))
    if len(compact) < 2:
        raise PatternError('The selected face meridian could not be sampled.')
    profile = []
    distance = 0.0
    previous = None
    for axial, radius in compact:
        if previous is not None:
            distance += math.sqrt((axial - previous[0]) ** 2 +
                                  (radius - previous[1]) ** 2)
        profile.append((distance, axial, radius))
        previous = (axial, radius)
    if distance <= TOL:
        raise PatternError('The selected face meridian has zero length.')
    return tuple(profile)


def analyze_revolved_face(part, face):
    """Measure one exterior face generated by revolving a monotonic profile.

    The two end-circle centers recover the revolution axis.  A straight or
    curved meridian boundary supplies the radius-versus-axis profile, so the
    same frame supports cylinders, cones and smooth dumbbell-like bodies.
    """
    circles = []
    other_edges = []
    for edge_index in face.getEdges():
        edge = part.edges[edge_index]
        circle = _circle_from_edge(edge)
        if circle is None:
            other_edges.append(edge)
        else:
            circles.append((edge, circle))
    if len(circles) < 2:
        raise PatternError(
            'The picked face is not a supported surface of revolution. '
            'Pick one exterior revolved face bounded by two circular ends.')

    best = None
    for first in range(len(circles)):
        for second in range(first + 1, len(circles)):
            separation = _distance(circles[first][1][0],
                                   circles[second][1][0])
            if best is None or separation > best[0]:
                best = (separation, circles[first], circles[second])
    if best is None or best[0] <= 1.0e-8:
        raise PatternError('Two distinct end circles are required to recover '
                           'the revolution axis.')
    first_circle = best[1][1]
    second_circle = best[2][1]
    axis_point = first_circle[0]
    axis = unit(sub(second_circle[0], first_circle[0]), 'revolution axis')

    try:
        first_point = part.DatumPointByCoordinate(coords=axis_point)
        second_point = part.DatumPointByCoordinate(coords=second_circle[0])
        axis_feature = part.DatumAxisByTwoPoint(
            point1=part.datums[first_point.id],
            point2=part.datums[second_point.id])
    except Exception:
        raise PatternError('A datum axis could not be created for the picked '
                           'surface of revolution.')

    angular_span = sum(item[1][2] for item in circles) / len(circles)
    angular_span = clamp(angular_span, 1.0e-8, 2.0 * math.pi)
    face_point = _point_on(face)
    boundary_directions = []
    for vertex_index in face.getVertices():
        point = _point_on(part.vertices[vertex_index])
        radial = radial_component(point, axis_point, axis)
        if norm(radial) > 1.0e-8:
            boundary_directions.append(unit(radial))
    for edge, unused_circle in circles:
        point = _point_on(edge)
        radial = radial_component(point, axis_point, axis)
        if norm(radial) > 1.0e-8:
            boundary_directions.append(unit(radial))
    radial_start, tangent_start, angular_span = _sector_basis(
        axis, axis_point, face_point, boundary_directions, angular_span)

    samples = []
    widest_range = -1.0
    for edge in other_edges:
        candidate = _sample_meridian_edge(part, edge)
        if len(candidate) < 2:
            continue
        coordinates = [axis_coordinate(point, axis_point, axis)
                       for point in candidate]
        axial_range = max(coordinates) - min(coordinates)
        if axial_range > widest_range:
            widest_range = axial_range
            samples = candidate
    # Exact circular data stabilizes both endpoints and also handles a full
    # cylinder/cone whose kernel does not expose its seam as a usable edge.
    samples.extend((item[1][0][0] + item[1][1] * radial_start[0],
                    item[1][0][1] + item[1][1] * radial_start[1],
                    item[1][0][2] + item[1][1] * radial_start[2])
                   for item in circles)
    profile = _profile_from_samples(samples, axis_point, axis)
    radii = [item[2] for item in profile]
    reference = profile_state({'profile': profile,
                               'meridional_length': profile[-1][0]},
                              0.5 * profile[-1][0])
    radius = reference['radius']
    variation = max(radii) - min(radii)
    surface_kind = ('cylinder' if variation <=
                    max(1.0e-6, radius * 1.0e-5) else 'revolved')
    return {
        'axis_point': axis_point,
        'axis': axis,
        'radius': radius,
        'minimum_radius': min(radii),
        'axial_min': profile[0][1],
        'axial_max': profile[-1][1],
        'axial_length': profile[-1][1] - profile[0][1],
        'meridional_length': profile[-1][0],
        'profile': profile,
        'surface_kind': surface_kind,
        'radial_start': radial_start,
        'tangent_start': tangent_start,
        'angular_span': angular_span,
        'axis_datum_id': axis_feature.id,
    }


def analyze_cylindrical_face(part, face):
    """Backward-compatible alias; now accepts general revolved faces."""
    return analyze_revolved_face(part, face)


def radial_at(frame, theta):
    return unit(rotate_in_plane(frame['radial_start'],
                                frame['tangent_start'], theta))


def tangent_at(frame, theta):
    u = frame['radial_start']
    v = frame['tangent_start']
    return unit(add(scale(u, -math.sin(theta)),
                    scale(v, math.cos(theta))))


def axis_point_at(frame, axial_coordinate_value):
    return add(frame['axis_point'],
               scale(frame['axis'], axial_coordinate_value))


def surface_point(frame, theta, axial_coordinate_value, radial_offset=0.0):
    return add(axis_point_at(frame, axial_coordinate_value),
               scale(radial_at(frame, theta),
                     frame['radius'] + radial_offset))


def profile_state(frame, meridional_coordinate):
    """Interpolate radius, axial position, tangent and normal on a meridian."""
    if 'profile' not in frame:
        start = frame.get('axial_min', 0.0)
        length = frame['axial_length']
        profile = ((0.0, start, frame['radius']),
                   (length, start + length, frame['radius']))
    else:
        profile = frame['profile']
    total = profile[-1][0]
    value = clamp(float(meridional_coordinate), 0.0, total)
    segment = len(profile) - 2
    for index in range(len(profile) - 1):
        if value <= profile[index + 1][0] + 1.0e-12:
            segment = index
            break
    first = profile[segment]
    second = profile[segment + 1]
    ds = second[0] - first[0]
    ratio = 0.0 if ds <= TOL else (value - first[0]) / ds
    axial = first[1] + ratio * (second[1] - first[1])
    radius = first[2] + ratio * (second[2] - first[2])
    dt = (second[1] - first[1]) / max(ds, TOL)
    dr = (second[2] - first[2]) / max(ds, TOL)
    tangent_length = math.sqrt(dt * dt + dr * dr)
    if tangent_length <= TOL:
        raise PatternError('A sampled meridian segment has zero length.')
    dt /= tangent_length
    dr /= tangent_length
    normal_radius = dt
    normal_axial = -dr
    if normal_radius < 0.0:
        normal_radius = -normal_radius
        normal_axial = -normal_axial
    return {'s': value, 'axial': axial, 'radius': radius,
            'tangent_radius': dr, 'tangent_axial': dt,
            'normal_radius': normal_radius,
            'normal_axial': normal_axial}


def surface_point_profile(frame, theta, meridional_coordinate,
                          normal_offset=0.0):
    state = profile_state(frame, meridional_coordinate)
    radial = radial_at(frame, theta)
    return add(
        add(frame['axis_point'],
            scale(frame['axis'], state['axial'] +
                  normal_offset * state['normal_axial'])),
        scale(radial, state['radius'] +
              normal_offset * state['normal_radius']))


def surface_basis(frame, theta, meridional_coordinate):
    """Return circumferential, meridional and outward-normal directions."""
    state = profile_state(frame, meridional_coordinate)
    radial = radial_at(frame, theta)
    circumferential = tangent_at(frame, theta)
    meridional = unit(add(
        scale(radial, state['tangent_radius']),
        scale(frame['axis'], state['tangent_axial'])))
    normal = unit(add(
        scale(radial, state['normal_radius']),
        scale(frame['axis'], state['normal_axial'])))
    # Imported sectors can have a clockwise parameter direction. Cubes only
    # need a proper right-handed basis, so correct the first axis if needed.
    if dot(cross(circumferential, meridional), normal) < 0.0:
        circumferential = scale(circumferential, -1.0)
    return circumferential, meridional, normal


def groove_centers(frame, rows, columns, axial_size, circum_size):
    rows = int(rows)
    columns = int(columns)
    axial_size = float(axial_size)
    circum_size = float(circum_size)
    if rows < 1 or columns < 1:
        raise PatternError('Groove rows and columns must be positive.')
    if axial_size <= 0.0 or circum_size <= 0.0:
        raise PatternError('Groove width and length must be positive.')

    length = frame['axial_length']
    span = frame['angular_span']
    radius = frame['radius']
    axial_margin = max(0.02 * length, 0.5 * axial_size)
    angular_size = circum_size / radius
    angular_margin = max(0.02 * span, 0.5 * angular_size)
    axial_available = length - 2.0 * axial_margin
    angular_available = span - 2.0 * angular_margin
    if axial_available <= 0.0 or angular_available <= 0.0:
        raise PatternError('The groove dimensions do not fit on the selected '
                           'cylindrical face.')
    axial_pitch = axial_available / rows
    angular_pitch = angular_available / columns
    if axial_size > axial_pitch * (1.0 + 1.0e-7):
        raise PatternError('Groove axial length is greater than the available '
                           'row pitch. Reduce the size or row count.')
    if angular_size > angular_pitch * (1.0 + 1.0e-7):
        raise PatternError('Groove circumferential width is greater than the '
                           'available column pitch. Reduce it or the columns.')

    centers = []
    for row in range(rows):
        axial = (frame['axial_min'] + axial_margin +
                 (row + 0.5) * axial_pitch)
        for column in range(columns):
            theta = angular_margin + (column + 0.5) * angular_pitch
            centers.append((theta, axial, row, column))
    return centers


def chocolate_groove_layout(
        frame, rows, columns, block_axial_length, block_circum_width,
        transverse_groove_width, longitudinal_groove_width,
        axial_offset=0.0, angular_offset_degrees=0.0, fit_to_face=False,
        end_margin=0.0):
    """Lay out continuous crossed grooves that leave raised curved blocks.

    Rows/columns are counts of the raised chocolate-like blocks.  The patch is
    centered within the picked cylindrical face, then moved by the two offset
    inputs. Circumferential dimensions are arc lengths at the outer radius.
    """
    rows = int(rows)
    columns = int(columns)
    block_axial_length = float(block_axial_length)
    block_circum_width = float(block_circum_width)
    transverse_groove_width = float(transverse_groove_width)
    longitudinal_groove_width = float(longitudinal_groove_width)
    axial_offset = float(axial_offset)
    angular_offset = math.radians(float(angular_offset_degrees))
    end_margin = float(end_margin)
    if rows < 1 or columns < 1:
        raise PatternError('Raised-block rows and columns must be positive.')
    if rows == 1 and columns == 1:
        raise PatternError('At least two raised blocks are required to create '
                           'a groove.')
    if transverse_groove_width <= 0.0 or longitudinal_groove_width <= 0.0:
        raise PatternError('Groove widths must be positive.')
    if end_margin < 0.0:
        raise PatternError('End margin cannot be negative.')

    radius = frame['radius']
    meridional_length = frame.get('meridional_length',
                                  frame['axial_length'])
    usable_length = meridional_length - 2.0 * end_margin
    if usable_length <= 0.0:
        raise PatternError('The two end margins consume the selected face.')
    if bool(fit_to_face):
        block_axial_length = (
            usable_length -
            (rows - 1) * transverse_groove_width) / rows
        block_circum_width = (
            radius * frame['angular_span'] -
            (columns - 1) * longitudinal_groove_width) / columns
        axial_offset = 0.0
        angular_offset = 0.0
        if block_axial_length <= 0.0 or block_circum_width <= 0.0:
            raise PatternError(
                'Groove widths consume the selected face. Reduce groove '
                'widths or the raised-block row/column count.')
    elif block_axial_length <= 0.0 or block_circum_width <= 0.0:
        raise PatternError('Block dimensions must be positive.')

    patch_axial_length = (rows * block_axial_length +
                          (rows - 1) * transverse_groove_width)
    patch_arc_width = (columns * block_circum_width +
                       (columns - 1) * longitudinal_groove_width)
    patch_angular_span = patch_arc_width / radius
    if patch_axial_length > usable_length + 1.0e-7:
        raise PatternError(
            'The chocolate groove patch is too long after applying the end '
            'margins. Reduce block length, groove width, row count, or the '
            'end margin.')
    if patch_angular_span > frame['angular_span'] + 1.0e-7:
        raise PatternError(
            'The chocolate groove patch is too wide for the picked face. '
            'Reduce block width, longitudinal groove width, or column count.')

    if bool(fit_to_face):
        axial_start = end_margin
        axial_end = meridional_length - end_margin
    else:
        axial_center = 0.5 * meridional_length + axial_offset
        axial_start = axial_center - 0.5 * patch_axial_length
        axial_end = axial_center + 0.5 * patch_axial_length
    if (axial_start < end_margin - 1.0e-7 or
            axial_end > meridional_length - end_margin + 1.0e-7):
        raise PatternError('Meridional offset moves the groove patch into an '
                           'end margin or outside the picked face.')

    full = frame['angular_span'] >= 2.0 * math.pi - 1.0e-5
    if bool(fit_to_face):
        angular_start = 0.0
        angular_end = frame['angular_span']
    else:
        base_center = 0.0 if full else 0.5 * frame['angular_span']
        angular_center = base_center + angular_offset
        angular_start = angular_center - 0.5 * patch_angular_span
        angular_end = angular_center + 0.5 * patch_angular_span
    if not full and (angular_start < -1.0e-7 or
                     angular_end > frame['angular_span'] + 1.0e-7):
        raise PatternError('Angular offset moves the groove patch outside the '
                           'picked face.')

    transverse_centers = []
    for boundary in range(1, rows):
        transverse_centers.append(
            axial_start + boundary * block_axial_length +
            (boundary - 0.5) * transverse_groove_width)

    block_angle = block_circum_width / radius
    groove_angle = longitudinal_groove_width / radius
    longitudinal_centers = []
    for boundary in range(1, columns):
        longitudinal_centers.append(
            angular_start + boundary * block_angle +
            (boundary - 0.5) * groove_angle)

    block_centers = []
    for row in range(rows):
        axial = (axial_start + (row + 0.5) * block_axial_length +
                 row * transverse_groove_width)
        for column in range(columns):
            theta = (angular_start + (column + 0.5) * block_angle +
                     column * groove_angle)
            block_centers.append((theta, axial, row, column))
    return {
        'rows': rows,
        'columns': columns,
        'axial_start': axial_start,
        'axial_end': axial_end,
        'axial_length': patch_axial_length,
        'meridional_start': axial_start,
        'meridional_end': axial_end,
        'meridional_length': patch_axial_length,
        'end_margin': end_margin,
        'angular_start': angular_start,
        'angular_end': angular_end,
        'angular_span': patch_angular_span,
        'arc_width': patch_arc_width,
        'block_axial_length': block_axial_length,
        'block_circum_width': block_circum_width,
        'fit_to_face': bool(fit_to_face),
        'transverse_centers': tuple(transverse_centers),
        'longitudinal_centers': tuple(longitudinal_centers),
        'transverse_groove_width': transverse_groove_width,
        'longitudinal_groove_width': longitudinal_groove_width,
        'longitudinal_groove_angle': groove_angle,
        'block_centers': tuple(block_centers),
    }


def _attachment_dimensions(body_size, body_kind, meridional_size=None,
                           circumferential_size=None):
    body_size = float(body_size)
    if body_kind == 'sphere':
        return body_size, body_size, body_size
    meridional = (body_size if meridional_size is None or
                  float(meridional_size) <= 0.0 else
                  float(meridional_size))
    circumferential = (body_size if circumferential_size is None or
                       float(circumferential_size) <= 0.0 else
                       float(circumferential_size))
    return meridional, circumferential, body_size


def _attachment_spacing(frame, body_size, packing, body_kind, embed,
                        meridional_coordinate=None, meridional_size=None,
                        circumferential_size=None):
    """Return angular/meridional pitches and angular half-footprint.

    ``embed`` may be negative. A negative value means a positive clearance
    from the picked face, which is used by the detached surface-cover mode.
    """
    body_size = float(body_size)
    packing = int(packing)
    if body_size <= 0.0:
        raise PatternError('Attachment size must be positive.')
    if packing not in (0, 1, 2):
        raise PatternError('Packing must be aligned, alternating staggered, '
                           'or progressively shifted.')
    meridional, circumferential, normal_size = _attachment_dimensions(
        body_size, body_kind, meridional_size, circumferential_size)
    coordinate = (0.5 * frame.get('meridional_length', frame['axial_length'])
                  if meridional_coordinate is None else
                  float(meridional_coordinate))
    local_radius = profile_state(frame, coordinate)['radius']
    center_radius = local_radius + 0.5 * normal_size - float(embed)
    if center_radius <= 0.5 * normal_size:
        raise PatternError('Attachment size/embed is invalid for this radius.')
    if body_kind == 'sphere':
        angular_pitch = 2.0 * math.asin(clamp(
            body_size / (2.0 * center_radius), 0.0, 1.0))
        axial_pitch = (math.sqrt(3.0) * 0.5 * body_size
                       if packing == 1 else body_size)
        angular_half_footprint = math.asin(clamp(
            0.5 * body_size / center_radius, 0.0, 1.0))
    else:
        angular_pitch = circumferential / local_radius
        axial_pitch = meridional
        angular_half_footprint = math.atan2(
            0.5 * circumferential,
            max(TOL, local_radius - float(embed)))
    return angular_pitch, axial_pitch, angular_half_footprint


def attachment_counts_to_fill(frame, body_size, packing, body_kind, embed,
                              meridional_size=None,
                              circumferential_size=None):
    """Calculate the largest complete row/column grid fitting the face."""
    meridional, unused_circumferential, unused_normal = (
        _attachment_dimensions(body_size, body_kind, meridional_size,
                               circumferential_size))
    unused_angle, axial_pitch, unused_half = _attachment_spacing(
        frame, body_size, packing, body_kind, embed,
        meridional_size=meridional_size,
        circumferential_size=circumferential_size)
    meridional_length = frame.get('meridional_length',
                                  frame['axial_length'])
    rows = int(math.floor(
        max(0.0, meridional_length - meridional) / axial_pitch +
        1.0e-9)) + 1
    if rows < 1 or meridional > meridional_length + 1.0e-7:
        raise PatternError('One attachment does not fit along the selected '
                           'face meridian.')
    first_s = 0.5 * (meridional_length - (rows - 1) * axial_pitch)
    row_columns = []
    for row in range(rows):
        coordinate = first_s + row * axial_pitch
        angular_pitch, unused_axial, angular_half = _attachment_spacing(
            frame, body_size, packing, body_kind, embed, coordinate,
            meridional_size, circumferential_size)
        available = frame['angular_span'] - 2.0 * angular_half
        if available < -1.0e-7:
            raise PatternError('One attachment does not fit across the '
                               'selected face circumference.')
        row_columns.append(int(math.floor(
            max(0.0, available) / angular_pitch + 1.0e-9)) + 1)
    # One conservative count keeps every row complete while each row uses
    # its own local-radius angular pitch. This removes the stretched look on
    # cones and dumbbell profiles without clipping a small-radius row.
    return max(1, rows), max(1, min(row_columns))


def attachment_centers(frame, rows, columns, body_size, packing, body_kind,
                       embed, meridional_size=None,
                       circumferential_size=None,
                       progressive_shift_degrees=0.0, layers=1,
                       layer_packing=0):
    """Create a centered, close-packed surface patch.

    packing: 0=aligned, 1=alternating half-pitch, 2=progressive angle.
    body_kind is 'cube' or 'sphere'.
    """
    rows = int(rows)
    columns = int(columns)
    body_size = float(body_size)
    packing = int(packing)
    layers = int(layers)
    layer_packing = int(layer_packing)
    if rows < 1 or columns < 1:
        raise PatternError('Attachment rows and columns must be positive.')
    if layers < 1:
        raise PatternError('Attachment layer count must be positive.')
    if layer_packing not in (0, 1):
        raise PatternError('Layer packing must be direct or staggered.')
    meridional_size_value, circumferential_size_value, normal_size = (
        _attachment_dimensions(body_size, body_kind, meridional_size,
                               circumferential_size))
    unused_angle, axial_pitch, unused_half = _attachment_spacing(
        frame, body_size, packing, body_kind, embed,
        meridional_size=meridional_size,
        circumferential_size=circumferential_size)

    meridional_length = frame.get('meridional_length',
                                  frame['axial_length'])
    center_axial_height = (rows - 1) * axial_pitch
    if (center_axial_height + meridional_size_value >
            meridional_length + 1.0e-7):
        raise PatternError('The close-packed rows do not fit along the '
                           'selected revolved face.')
    axial0 = 0.5 * (meridional_length - center_axial_height)
    base_centers = []
    full = frame['angular_span'] >= 2.0 * math.pi - 1.0e-5
    for row in range(rows):
        coordinate = axial0 + row * axial_pitch
        angular_pitch, unused_axial, angular_half_footprint = (
            _attachment_spacing(
                frame, body_size, packing, body_kind, embed, coordinate,
                meridional_size, circumferential_size))
        center_angular_width = (columns - 1) * angular_pitch
        if (center_angular_width + 2.0 * angular_half_footprint >
                frame['angular_span'] + 1.0e-7):
            raise PatternError('The requested columns do not fit at row %d '
                               'because its local radius is smaller.' %
                               (row + 1))
        theta0 = (0.5 * frame['angular_span'] -
                  0.5 * center_angular_width)
        if packing == 1:
            raw_shift = 0.5 * angular_pitch if row % 2 else 0.0
        elif packing == 2:
            raw_shift = math.radians(float(progressive_shift_degrees)) * row
        else:
            raw_shift = 0.0
        if full:
            shift = raw_shift
        else:
            low_shift = angular_half_footprint - theta0
            high_shift = (frame['angular_span'] - angular_half_footprint -
                          (theta0 + center_angular_width))
            shift = clamp(raw_shift, low_shift, high_shift)
        for column in range(columns):
            theta = theta0 + column * angular_pitch + shift
            if full:
                theta %= frame['angular_span']
            base_centers.append((theta, coordinate, row, column,
                                 angular_pitch, axial_pitch,
                                 angular_half_footprint))

    centers = []
    for layer in range(layers):
        stagger_layer = layer_packing == 1 and layer % 2 == 1
        for item in base_centers:
            theta, coordinate, row, column = item[:4]
            angular_pitch, row_pitch, angular_half = item[4:]
            layer_coordinate = coordinate
            layer_theta = theta
            if stagger_layer:
                layer_theta += 0.5 * angular_pitch
                layer_coordinate += 0.5 * row_pitch
            if (layer_coordinate - 0.5 * meridional_size_value < -1.0e-7 or
                    layer_coordinate + 0.5 * meridional_size_value >
                    meridional_length + 1.0e-7):
                continue
            if full:
                layer_theta %= frame['angular_span']
            elif (layer_theta - angular_half < -1.0e-7 or
                  layer_theta + angular_half >
                  frame['angular_span'] + 1.0e-7):
                continue
            if body_kind == 'sphere' and layer_packing == 1:
                circum_shift = 0.5 * body_size
                meridian_shift = 0.5 * row_pitch
                normal_pitch = 0.995 * math.sqrt(max(
                    0.25 * body_size * body_size,
                    body_size * body_size - circum_shift * circum_shift -
                    meridian_shift * meridian_shift))
            else:
                normal_pitch = normal_size
            centers.append((layer_theta, layer_coordinate, row, column,
                            layer, layer * normal_pitch))
    return centers


def box_orientation(frame, theta):
    """Return target global directions for local box X, Y and Z."""
    coordinate = 0.5 * frame.get('meridional_length', frame['axial_length'])
    return surface_basis(frame, theta, coordinate)


def matrix_axis_angle(x_axis, y_axis, z_axis):
    """Convert a right-handed basis (matrix columns) to axis/angle."""
    matrix = ((x_axis[0], y_axis[0], z_axis[0]),
              (x_axis[1], y_axis[1], z_axis[1]),
              (x_axis[2], y_axis[2], z_axis[2]))
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    angle = math.acos(clamp(0.5 * (trace - 1.0), -1.0, 1.0))
    if angle < 1.0e-10:
        return (1.0, 0.0, 0.0), 0.0
    if abs(math.pi - angle) < 1.0e-6:
        xx = max(0.0, 0.5 * (matrix[0][0] + 1.0))
        yy = max(0.0, 0.5 * (matrix[1][1] + 1.0))
        zz = max(0.0, 0.5 * (matrix[2][2] + 1.0))
        axis = [math.sqrt(xx), math.sqrt(yy), math.sqrt(zz)]
        if matrix[0][1] < 0.0:
            axis[1] = -axis[1]
        if matrix[0][2] < 0.0:
            axis[2] = -axis[2]
        return unit(tuple(axis)), math.degrees(angle)
    axis = (matrix[2][1] - matrix[1][2],
            matrix[0][2] - matrix[2][0],
            matrix[1][0] - matrix[0][1])
    return unit(axis), math.degrees(angle)


def suggested_embed(body_size, radius, body_kind, fraction,
                    footprint_size=None):
    fraction = float(fraction)
    if not 0.0 < fraction < 0.5:
        raise PatternError('Embed fraction must be between 0 and 0.5.')
    embed = fraction * body_size
    footprint = (body_size if footprint_size is None or
                 float(footprint_size) <= 0.0 else
                 float(footprint_size))
    if body_kind == 'cube' and footprint < 2.0 * radius:
        sagitta = radius - math.sqrt(max(
            0.0, radius * radius - 0.25 * footprint * footprint))
        embed = max(embed, sagitta + 1.0e-5 * body_size)
    return min(0.45 * body_size, embed)
