import mongoengine as me
from Viewpoint import Viewpoint
from Point2d import Point2d
import xml.etree.ElementTree as ET


class Annotation(me.EmbeddedDocument):
    label = me.StringField(max_length=100, default='')
    polygon = me.PolygonField()
    viewpoints = me.ListField(me.EmbeddedDocumentField(Viewpoint))

    def to_xml(self):
        attribs = {x: self.__getattribute__(x) for x in self._fields}
        print(attribs['label'])
        viewpoints = attribs.pop('viewpoints')
        polygon = attribs.pop('polygon')
        attribs = {x: str(attribs[x]) for x in attribs}
        root = ET.Element('ANNOTATION', attrib=attribs)
        for point in viewpoints:
            root.append(point.to_xml())
        if polygon:  # check if polygon was set (may not be for )
            poly = ET.SubElement(root, 'PRECISEPOLYGON')
            if type(polygon) == dict:  # the polygon was loaded from database
                for point in polygon['coordinates'][0][:-1]: #omit last point wich is also the first one (geojson specific stuff)
                    poi = Point2d(x=point[0],
                                  y=point[1])
                    poly.append(poi.to_xml())
            elif type(polygon) == list:  # the polygon was created "manually"
                for point in polygon[0][:-1]: #omit last point wich is also the first one (geojson specific stuff)
                    poi = Point2d(x=point[0],
                                  y=point[1])
                    poly.append(poi.to_xml())

            gen = ET.SubElement(poly, 'GENERATOR')
            gen.text = 'Kbase'
            time = ET.SubElement(poly, 'TIMESTAMP')
            inserted = ET.SubElement(time, 'INSERTED', {'value': '1'})
            updated = ET.SubElement(time, 'UPDATED', {'value': '1'})
        else:
            print('Warning: Annotation with label \"' + attribs['label'] + '\" has no PrecisePolygon')

        gen = ET.SubElement(root, 'GENERATOR')
        gen.text = 'Kbase'
        time = ET.SubElement(root, 'TIMESTAMP')
        inserted = ET.SubElement(time, 'INSERTED', {'value': '1'})
        updated = ET.SubElement(time, 'UPDATED', {'value': '1'})
        return root

    @classmethod
    def from_xml(cls, xml_tree):
        anno = Annotation()
        anno.label = xml_tree.get('label')

        vps = []
        for child in xml_tree.getchildren():
            if child.tag.lower() == Viewpoint.get_tag().lower():
                vps.append(Viewpoint.from_xml(child))
            elif child.tag.lower() == 'precisepolygon':
                points = []
                for potential_point in child.getchildren():
                    if potential_point.tag.lower() == Point2d.get_tag().lower():
                        p = Point2d.from_xml(potential_point)
                        points.append([p.x, p.y])
                points.append(points[0])
                anno.polygon = [points]

        anno.viewpoints = vps

        return anno

    @classmethod
    def get_tag(cls):
        return 'ANNOTATION'

    def __str__(self):
        label_str = 'label : "' + str(self.label) + '"'
        viewpoints_str = 'viewpoints: '
        for vp in self.viewpoints:
            viewpoints_str += str(vp)
        polygon_str = 'polygon: ' + str(self.polygon)
        return label_str + '; ' + viewpoints_str + '; ' + polygon_str
