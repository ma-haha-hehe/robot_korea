"""Compliant stud fit for the self-authored hollow brick model.

Parameters describe a simulation approximation, not measured ABS properties.
The fit uses contact forces and Coulomb friction; it creates no attachments.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class PlasticContact:
    sliding_friction: float = .3
    torsional_friction_m: float = .001
    rolling_friction_m: float = .00005
    contact_time_constant_s: float = .03
    damping_ratio: float = 1.

    def attributes(self):
        values = (self.sliding_friction, self.torsional_friction_m,
                  self.rolling_friction_m, self.contact_time_constant_s,
                  self.damping_ratio)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('plastic contact parameters must be finite and positive')
        return dict(solref=f'{self.contact_time_constant_s} {self.damping_ratio}',
                    solimp='.9 .95 .001', priority='2',
                    friction=' '.join(map(str, values[:3])))


def add_clutch_fit(body, brick_type, parameters=PlasticContact()):
    """Add internal ribs and lead-ins; return the required inline mesh assets.

    `body` must already contain the hollow model's walls, tubes and studs.
    Only the internal fit is compliant; the outer body retains hard contact.
    The returned asset elements belong in the scene's MJCF asset section.
    """
    if brick_type not in ('brick_2x2', 'brick_4x2'):
        raise ValueError(f'unsupported clutch part: {brick_type}')
    if any(g.get('name', '').startswith('clutch_') for g in body.findall('geom')):
        raise ValueError('clutch fit already present')
    contact = parameters.attributes()
    meshes = {}
    body_name = body.get('name', 'part')

    def prism(name, sections):
        name = f'clutch_{name}'
        vertices = [(x, y, z) for z, x0, x1, y0, y1 in sections
                    for x, y in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
        meshes[name] = ET.Element('mesh', name=name,
            vertex=' '.join(str(v) for point in vertices for v in point))
        return name

    radius = .00655
    tangent = radius * math.tan(math.pi / 16)
    for index, geom in enumerate(list(body.findall('geom'))):
        if geom.get('type') != 'box' or geom.get('euler') is None:
            continue
        angle = float(geom.get('euler').split()[2])
        pos = list(map(float, geom.get('pos').split()))
        size = list(map(float, geom.get('size').split()))
        pos[0] += .00045 * math.cos(angle)
        pos[1] += .00045 * math.sin(angle)
        geom.set('name', f'clutch_{body_name}_tube_{index}')
        geom.set('pos', ' '.join(map(str, pos)))
        # A single convex hull joins the lead-in to the straight wall. Two
        # overlapping geoms expose an internal end face to the contact solver.
        mesh = prism('tube', (
            (-size[2], -size[0]-.00045, size[0]-.00045, -tangent, tangent),
            (-size[2]+.002, -size[0], size[0], -tangent, tangent),
            (size[2], -size[0], size[0], -tangent, tangent)))
        geom.set('type', 'mesh')
        geom.set('mesh', mesh)
        del geom.attrib['size']
        geom.attrib.update(contact)

    # The 2 mm lead-in grows from clearance to 50 micrometres of nominal
    # interference against an aligned stud. It avoids a square insertion edge.
    mesh = prism('rib', ((-.001, -.001, .001, .0011, .0012),
                 (.001, -.001, .001, -.0012, .0012),
                 (.0162, -.001, .001, -.0012, .0012)))
    hx = .016 if brick_type == 'brick_2x2' else .032
    xs = (-.008, .008) if hx == .016 else (-.024, -.008, .008, .024)
    for sign in (-1, 1):
        for x in xs:
            ET.SubElement(body, 'geom', type='mesh', mesh=mesh,
                pos=f'{x} {sign*.01335} -.0176',
                euler=f'0 0 {0 if sign > 0 else math.pi}', **contact)
        for y in (-.008, .008):
            ET.SubElement(body, 'geom', type='mesh', mesh=mesh,
                pos=f'{sign*(hx-.00265)} {y} -.0176',
                euler=f'0 0 {-sign*math.pi/2}', **contact)
    return list(meshes.values())
