import mongoengine as me
import xml.etree.ElementTree as ET


class Point2d(me.EmbeddedDocument):
    x = me.FloatField(default=0.0)
    y = me.FloatField(default=0.0)

    def to_xml(self):
        attribs = {x: self.__getattribute__(x) for x in self._fields}
        attribs = {x: str(attribs[x]) for x in attribs}
        root = ET.Element(Point2d.get_tag(), attrib=attribs)

        gen = ET.SubElement(root, 'GENERATOR')
        gen.text = 'Kbase'
        time = ET.SubElement(root, 'TIMESTAMP')
        inserted = ET.SubElement(time, 'INSERTED', {'value': '1'})
        updated = ET.SubElement(time, 'UPDATED', {'value': '1'})
        return root

    @classmethod
    def from_xml(cls, xml_tree):
        posi = Point2d()
        posi.x = float(xml_tree.get('x'))
        posi.y = float(xml_tree.get('y'))

        return posi

    @classmethod
    def get_tag(cls):
        return 'POINT2D'

    def __str__(self):
        return 'x: ' + str(self.x) + ', y: ' + str(self.y)
