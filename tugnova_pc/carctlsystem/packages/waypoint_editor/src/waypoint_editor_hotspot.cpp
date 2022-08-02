#include <waypoint_editor.h>

namespace WaypointEditor {
    Hotspot::Hotspot() {
        setData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        isInitialized = true;
    }

    void Hotspot::setData(
        const double _x,
        const double _y,
        const double _z,
        const double _qx,
        const double _qy,
        const double _qz,
        const double _qw
    ) {
        x = _x;
        y = _y;
        z = _z;

        tf::Quaternion q(_qx, _qy, _qz, _qw);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        isInitialized = false;
    }

    double Hotspot::getX() { return x; }
    double Hotspot::getY() { return y; }
    double Hotspot::getZ() { return z; }
    double Hotspot::getRoll() { return roll; }
    double Hotspot::getPitch() { return pitch; }
    double Hotspot::getYaw() { return yaw; }

    void Hotspot::clear() { 
        setData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        isInitialized = true;
    }
    bool Hotspot::hasData() { return !isInitialized; }
}
