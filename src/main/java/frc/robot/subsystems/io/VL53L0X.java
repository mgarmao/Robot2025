package frc.robot.subsystems.io;

import edu.wpi.first.wpilibj.I2C;

public class VL53L0X {
    private static final int VL53L0X_DEF_ADDR = 0x26;
    private static final int REG_MODEL_ID = 0xC0;

    private I2C i2c;

    private void VL53L0X(I2C.Port port) {
        i2c = new I2C(port, VL53L0X_DEF_ADDR);
    }

    private void writeRegister(int reg, int value) {
        byte[] data = new byte[] {(byte) reg, (byte) value};

        i2c.writeBulk(data);
    }

    private int readRegister(int reg) {
        byte[] data = new byte[1];

        i2c.write(reg, 0);
        i2c.readOnly(data, 1);

        return data[0] & 0xFF;
    }

    public boolean init() {
        int modelId = readRegister(REG_MODEL_ID);
        return modelId == 0xEE;
    }

    public int getDistance() {
        byte[] buffer = new byte[2];
        i2c.read(0x14, 2, buffer);
        int dist = ((buffer[0] << 8) | (buffer[1] & 0xFF));
        return dist;
    }
}