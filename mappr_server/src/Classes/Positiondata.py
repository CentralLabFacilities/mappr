import mongoengine as me
import xml.etree.ElementTree as ET
from Point2d import Point2d


class Positiondata(me.EmbeddedDocument):
    theta = me.FloatField(default=0.0)
    frameid = me.StringField(max_length=20, default='map')
    point2d = me.EmbeddedDocumentField(Point2d)

    def to_xml(self):
        attribs = {x: self.__getattribute__(x) for x in self._fields}
        poi = attribs.pop('point2d')
        attribs = {x: str(attribs[x]) for x in attribs}
        root = ET.Element(Positiondata.get_tag(), attrib=attribs)
        root.append(poi.to_xml())

        gen = ET.SubElement(root, 'GENERATOR')
        gen.text = 'Kbase'
        time = ET.SubElement(root, 'TIMESTAMP')
        inserted = ET.SubElement(time, 'INSERTED', {'value': '1'})
        updated = ET.SubElement(time, 'UPDATED', {'value': '1'})
        return root

    @classmethod
    def from_xml(cls, xml_tree):
        posi = Positiondata()
        posi.frameid = xml_tree.get('frameid')
        posi.theta = float(xml_tree.get('theta'))

        for potential_poi in xml_tree.getchildren():
            if potential_poi.tag.lower() == Point2d.get_tag().lower():
                posi.point2d = Point2d.from_xml(potential_poi)
        return posi

    @classmethod
    def get_tag(cls):
        return 'POSITIONDATA'

    def __str__(self):
        selfvalues = 'frameid: ' + str(self.frameid) + '; theta: ' + str(self.theta)
        point_str = 'point2d: ' + str(self.point2d)
        return selfvalues + '; ' + point_str