package org.firstinspires.ftc.teamcode.libs;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "MB1242 Ultrasonic Sensor", xmlTag = "MB1242")
 public class MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch>{
    public enum Register{
        TAKE_RANGE()
    }
    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.Unknown;
    }

    @Override
    protected synchronized boolean doInitialize(){
        return true;
    }
    @Override
    public String getDeviceName(){
        return "Maxbotix MB1242 Ultrasonic Sensor";
    }

    public MB1242(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0x71));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

}
