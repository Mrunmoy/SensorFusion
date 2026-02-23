# GPIO Interrupt HAL Design

## Purpose

Many sensors (MPU6050, QMC5883L, ADXL345) have hardware interrupt pins
that signal data-ready or other events. Instead of polling, the MCU can
configure a GPIO as an interrupt input and react immediately.

This HAL interface abstracts the MCU-side GPIO interrupt so drivers
remain platform-agnostic.

## Interface

```cpp
enum class GpioEdge { RISING, FALLING, BOTH };

class IGpioInterrupt {
public:
    virtual ~IGpioInterrupt() = default;

    using Callback = void(*)(void* context);

    virtual bool enable(GpioEdge edge, Callback cb, void* context) = 0;
    virtual bool disable() = 0;
};
```

### Design Rationale

- **One interface per pin**: Each `IGpioInterrupt` instance represents
  one physical GPIO interrupt line. The driver doesn't know the pin
  number — that's the platform's concern.

- **C function pointer + void* context**: Safe for ISR use on all
  platforms. No `std::function`, no virtual dispatch in the ISR itself.
  The callback should be minimal (set a flag, give a semaphore).

- **Optional**: Drivers accept `IGpioInterrupt*` (nullptr = polling mode).
  This preserves backward compatibility and allows both modes.

## How Drivers Use It

1. Driver configures the sensor's internal interrupt registers (e.g.
   MPU6050's INT_ENABLE at 0x38) via I2C.
2. Driver calls `intPin->enable(edge, callback, context)` to attach the
   MCU-side handler.
3. When sensor asserts its INT pin, the MCU GPIO fires the callback.
4. Callback sets a flag / gives a semaphore (ISR-safe, fast).
5. Application task wakes up and calls the driver's read method.

## Interrupt Support by Device

| Device | INT Pin | Trigger | Sensor Register |
|--------|---------|---------|-----------------|
| MPU6050 | INT | Data ready, motion | INT_ENABLE (0x38), INT_PIN_CFG (0x37) |
| QMC5883L | DRDY | Data ready | CTRL2 (0x0A) bit 0 |
| ADXL345 | INT1/INT2 | Data ready, tap, freefall, activity | INT_ENABLE (0x2E), INT_MAP (0x2F) |
| BMP180 | EOC (rare) | End of conversion | N/A — typically polled with delay |
| AD8232 | LO+/LO- | Leads-off detection | N/A — analog device |

BMP180 and AD8232 don't benefit from this interface (BMP180 uses timed
conversion, AD8232 is pure analog).

## Platform Implementation Examples

| Platform | Implementation |
|----------|---------------|
| ESP32 (ESP-IDF) | `gpio_install_isr_service()` + `gpio_isr_handler_add()` |
| nRF52 (Zephyr) | `gpio_pin_interrupt_configure()` + `gpio_init_callback()` |
| STM32 (HAL) | `HAL_GPIO_EXTI_Callback()` + NVIC config |
| Host (test) | `MockGpioInterrupt` with GMock |

## Mock Strategy

```cpp
class MockGpioInterrupt : public IGpioInterrupt {
    MOCK_METHOD(bool, enable, (GpioEdge, Callback, void*), (override));
    MOCK_METHOD(bool, disable, (), (override));
};
```

The mock can capture the callback and context, allowing tests to
simulate an interrupt by calling the captured callback directly.
