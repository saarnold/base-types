#ifndef VECTORFIELDVISUALIZATION_H
#define VECTORFIELDVISUALIZATION_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/Waypoint.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace osg {
    class Group;
}

namespace vizkit3d
{
    class VectorFieldVisualization
        : public vizkit3d::Vizkit3DPlugin< base::Vector3d >
	, public VizPluginAddType<base::samples::RigidBodyState>
        , boost::noncopyable
    {
    Q_OBJECT
    Q_PROPERTY(QColor Color READ getColor WRITE setColor)
    Q_PROPERTY(int MaxVectors READ getMaxNumberOfVectors WRITE setMaxNumberOfVectors)
    
    public:
        VectorFieldVisualization();
        ~VectorFieldVisualization();
        
        /**
         * Thread-safe call of 'updateDataIntern ( const base::Waypoint& data )'.
         */
        Q_INVOKABLE void updateData(base::Vector3d const &sample) {
            vizkit3d::Vizkit3DPlugin< base::Vector3d >::updateData(sample);
        }
        
	/**
         * Thread-safe call of 'updateDataIntern ( const base::Waypoint& data )'.
         */
        Q_INVOKABLE void updateOrigin(base::samples::RigidBodyState const &pose) {
            vizkit3d::Vizkit3DPlugin< base::Vector3d >::updateData(pose);
        }
        
    public slots:
        /**
         * Sets the color of all waypoints.
         */
        void setColor(QColor q_color);
        
        /**
         * Returns the current color of the waypoints.
         */
        QColor getColor() const;
	
	int getMaxNumberOfVectors(){return max_number_of_vectors;};
        void setMaxNumberOfVectors(int points){max_number_of_vectors = points;};
        
    protected:
        /**
         * OSG tree: Group <- Transformation <- Geode <- Sphere 
         *                                            <- Triangle
         */
        osg::ref_ptr<osg::Node> createMainNode();
        
        /**
         * Clears the group and redraws all waypoints.
         */
        void updateMainNode(osg::Node* node);

        /**
         * Clears the current list of waypoints and adds the new waypoint.
         */
        void updateDataIntern ( base::Vector3d const& data );
	
	/**
         * Clears the current list of waypoints and adds the new waypoint.
         */
        void updateDataIntern ( base::samples::RigidBodyState const& data );
	
	void addVector(osg::Group* group);
        
    private:
        osg::ref_ptr<osg::Group> group;
        struct Data;
        Data* p;
        osg::Vec4 color;
	int max_number_of_vectors;
	float vect_norm;
    };
}
#endif
