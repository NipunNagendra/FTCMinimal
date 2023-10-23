package org.firstinspires.ftc.teamcode.libs;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "MB1242 Ultrasonic Sensor", xmlTag = "MB1242")
 public class MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch>{
    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }
    public enum Register{
        INITIATE_WRITE(0xE0),
        WRITE_RANGE(0x51),
        INITIATE_READ(0xE1);

        public int bVal;

        Register(int bVal){
            this.bVal=bVal;
        }
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



    public MB1242(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0xE0));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow() {

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    public short readRange(){
        return readShort(Register.INITIATE_READ);
    }

    public short takeRange(){
        
    }

}
