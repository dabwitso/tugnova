#ifndef WAYPOINT_EDITOR_DATA_H
#define WAYPOINT_EDITOR_DATA_H

#include <waypoint_editor_base.h>

#include <carctl_msgs/output_target.h>

namespace WaypointEditor {
    class Data {
        public:
            Data(
                const int _waypoint_id,
                const double _x,
                const double _y,
                const double _z,
                const double _yaw,
                const int _sensor2d
            );

            int getWaypointId();
            double getX();
            double getY();
            double getZ();
            double getYaw();
            void setSensor2D(const int _sensor2d);
            int getSensor2D();

            void setStop(const int _stop);
            int getStop();

//shutter
            void setShutterPoint(const int _shutter_point); 
            int getShutterPoint();

            void setInputTargets(const std::vector<std::string> _input_targets);
            std::vector<std::string> getInputTargets();
            void setSplit(const int _split);
            int getSplit();
            void setPause(const int _pause);
            int getPause();
            void setOutputTargets(const std::vector<carctl_msgs::output_target> _output_targets);
            std::vector<carctl_msgs::output_target> getOutputTargets();
            void setHotspot(
                const double _x,
                const double _y,
                const double _z,
                const double _qx,
                const double _qy,
                const double _qz,
                const double _qw
            );
            void clearHotspot();
            WaypointEditor::Hotspot getHotspot();

            void setScore(const double _score);
            double getScore();
            void setFilteredScore(const double _filtered_score);
            double getFilteredScore();

            void setVelocity(const double _velocity);
            double getVelocity();
            void setDetectionRangeFrontL(const double _detection_range_front_l);
            double getDetectionRangeFrontL();
            void setDetectionRangeFrontR(const double _detection_range_front_r);
            double getDetectionRangeFrontR();
            void setDetectionRangeRearL(const double _detection_range_rear_l);
            double getDetectionRangeRearL();
            void setDetectionRangeRearR(const double _detection_range_rear_r);
            double getDetectionRangeRearR();
            void setWinkerPoint(const int _winker_point);
            int getWinkerPoint();
            void setAttentionBroadcast(const int _attention_broadcast);
            int getAttentionBroadcast();

            typedef std::shared_ptr<WaypointEditor::Data> Ptr;
        private:
            int waypoint_id;
            double x;
            double y;
            double z;
            double yaw;
            int sensor2d;

            WaypointEditor::Hotspot hotspot;

            double score;
            double filtered_score;

            int stop;
            

//shutter
            int shutter_point;
            std::vector<std::string> input_targets;
            int split;
            int pause;
            std::vector<carctl_msgs::output_target> output_targets;

            double velocity;
            double detection_range_front_l;
            double detection_range_front_r;
            double detection_range_rear_l;
            double detection_range_rear_r;
            int winker_point;
            int attention_broadcast;
    };
}

#endif /* WAYPOINT_EDITOR_DATA_H */
