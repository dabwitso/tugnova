#ifndef WAYPOINT_EDITOR_HOTSPOT_H
#define WAYPOINT_EDITOR_HOTSPOT_H

#include <waypoint_editor_base.h>

namespace WaypointEditor {
    class Hotspot {
        public:
            Hotspot();
            void setData(
                const double _x,
                const double _y,
                const double _z,
                const double _qx,
                const double _qy,
                const double _qz,
                const double _qw
            );
            double getX();
            double getY();
            double getZ();
            double getRoll();
            double getPitch();
            double getYaw();
            void clear();
            bool hasData();

        private:
            double x;
            double y;
            double z;
            double roll;
            double pitch;
            double yaw;
            bool isInitialized;
    };
}

#endif /* WAYPOINT_EDITOR_HOTSPOT_H */