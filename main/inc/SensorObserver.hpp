#pragma once

class SensorObserver {
public:
    virtual void onSensorUpdated() = 0;
    virtual ~SensorObserver() = default;
};
