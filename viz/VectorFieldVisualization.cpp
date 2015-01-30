#include "VectorFieldVisualization.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vizkit3d/Vizkit3DHelper.hpp>
#include <vizkit3d/ColorConversionHelper.hpp>

using namespace vizkit3d;

struct VectorFieldVisualization::Data {
    // Copy of the value given to updateDataIntern.
    // Making a copy is required because of how OSG works
    std::vector<base::Vector3d> vectors;
    base::samples::RigidBodyState pose;
};


VectorFieldVisualization::VectorFieldVisualization()
    : p(new Data), color(1.0f, 0.0f, 0.0f, 1.0f), max_number_of_vectors(1000), vect_norm(0)
{
    p->pose.initUnknown();
}

VectorFieldVisualization::~VectorFieldVisualization()
{
    delete p;
}

void VectorFieldVisualization::setColor(QColor q_color)
{
    color = osg::Vec4(q_color.redF(), q_color.greenF(), 
            q_color.blueF(), q_color.alphaF());
    setDirty();
}

QColor VectorFieldVisualization::getColor() const
{
    QColor q_color;
    q_color.setRgbF(color[0], color[1], color[2], color[3]);
    return q_color;
} 

osg::ref_ptr<osg::Node> VectorFieldVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();

    return mainNode;
}

void VectorFieldVisualization::updateMainNode ( osg::Node* node )
{
    addVector(node->asGroup());
}

void VectorFieldVisualization::updateDataIntern(const base::Vector3d& data)
{
    p->vectors.push_back(data);
}

void VectorFieldVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    p->pose = data;
}

void VectorFieldVisualization::addVector(osg::Group* group)
{
    // remove old vectors if necessary
    if(group->getNumChildren() > max_number_of_vectors)
    {
	group->removeChildren(0, group->getNumChildren() - max_number_of_vectors);
    }
    
    // add new vectors
    for(std::vector<base::Vector3d>::iterator it = p->vectors.begin(); it != p->vectors.end(); ++it)
    {
	if(!(*it).isZero())
	{
	    // Create lines
	    osg::ref_ptr<osg::Geometry> line_geometry = new osg::Geometry();
	    osg::ref_ptr<osg::Vec3Array> line_vertices = new osg::Vec3Array();
	    line_vertices->push_back(eigenVectorToOsgVec3(p->pose.position));
	    base::Vector3d vec = p->pose.orientation * (*it);
	    vec.z() = 0.0;
	    vec.normalize();
	    line_vertices->push_back(eigenVectorToOsgVec3(p->pose.position + vec));
	    line_geometry->setVertexArray(line_vertices);
	    osg::ref_ptr<osg::DrawArrays> drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, line_vertices->size() );
	    line_geometry->addPrimitiveSet(drawArrays);
	    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	    if(vect_norm == 0.0)
		vect_norm = (*it).norm();
	    float scale = (*it).norm()/vect_norm;
	    vizkit3d::hslToRgb( scale, 1.0, 0.6, color.x(), color.y(), color.z() );
	    colors->push_back( color );
	    line_geometry->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);
	    
	    // Add drawables to the geode.
	    osg::ref_ptr<osg::Geode> waypoint_geode = new osg::Geode();
	    waypoint_geode->addDrawable(line_geometry);
	    
	    // Adds the waypoints to the main node.
	    group->addChild(waypoint_geode);
	}
    }
    p->vectors.clear();
}
