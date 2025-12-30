package org.firstinspires.ftc.teamcode.configuration;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.TimelockedServo;

/**
 * Represents a hardware device configuration with both its type and string
 * name.
 * This allows for type-safe hardware retrieval from the HardwareMap.
 */
public class HardwareConfig {
	private final Class<?> deviceType;
	private final String deviceName;

	/**
	 * Creates a new HardwareConfig with the specified device type and name.
	 *
	 * @param deviceType The class type of the hardware device
	 * @param deviceName The string name of the device in the hardware map
	 */
	public HardwareConfig(Class<?> deviceType, String deviceName) {
		this.deviceType = deviceType;
		this.deviceName = deviceName;
	}
	
	/**
	 * Retrieves the hardware device from the given HardwareMap.
	 * This is the "frozen invocation" - it encapsulates both the type and name,
	 * so you can simply call get() instead of hw.get(Class, String).
	 *
	 * @param hardwareMap The hardware map to retrieve the device from
	 * @param <T>         The type of hardware device to return
	 * @return The hardware device instance
	 */
	@SuppressWarnings("unchecked")
	public <T> T fromHardwareMap(HardwareMap hardwareMap) {
		return (T) hardwareMap.get(deviceType, deviceName);
	}
	
	/**
	 * Gets the string name of the device.
	 * Useful for cases where only the name is needed (e.g., pedropathing
	 * builders).
	 *
	 * @return The device name string
	 */
	public String getName() {
		return deviceName;
	}
	
	/**
	 * Convenience: wrap a ServoImplEx in a TimelockedServo with cooldownMs.
	 */
	public TimelockedServo asTimelockedServo(HardwareMap hardwareMap, long cooldownMs) {
		Object raw = hardwareMap.get(deviceType, deviceName);
		if (!(raw instanceof com.qualcomm.robotcore.hardware.ServoImplEx)) {
			throw new IllegalArgumentException("Device " + deviceName + " is not a ServoImplEx");
		}
		return new TimelockedServo((com.qualcomm.robotcore.hardware.ServoImplEx) raw, cooldownMs);
	}
	
	/**
	 * Gets the device type class.
	 *
	 * @return The device type class
	 */
	public Class<?> getType() {
		return deviceType;
	}
}
