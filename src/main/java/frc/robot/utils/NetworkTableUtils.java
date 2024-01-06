package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableUtils {
    private NetworkTable table;

    public NetworkTableUtils(String table) {
        this.table = NetworkTableInstance.getDefault().getTable(table);
    }

    public NetworkTable getTable() {
        return this.table;
    }

    public double getDouble(String key, double defaultValue) {
        return this.table.getEntry(key).getDouble(defaultValue);
    }

    public String getString(String key, String defaultValue) {
        return this.table.getEntry(key).getString(defaultValue);
    }

    public double[] getDoubleArray(String key, double[] doubles) {
        return this.table.getEntry(key).getDoubleArray(doubles);
    }

    public void setDouble(String key, double value) {
        this.table.getEntry(key).setDouble(value);
    }

    public void setString(String key, String value) {
        this.table.getEntry(key).setString(value);
    }

    public <T> T getEntry(String key, T value) {
        if (this.table.getEntry(key).exists()) {
            if (value instanceof Double) {
                return (T) (Object) getDouble(key, (double) value);
            } else if (value instanceof String) {
                return (T) (Object) getString(key, (String) value);
            } else {
                throw new IllegalArgumentException("Invalid value type");
            }
        } else {
            throw new IllegalArgumentException("Invalid key");
        }
    }

    public <T> void setEntry(String key, T value) {
        if (this.table.getEntry(key).exists()) {
            if (value instanceof Double) {
                setDouble(key, (double) value);
            } else if (value instanceof String) {
                setString(key, (String) value);
            } else {
                throw new IllegalArgumentException("Invalid value type");
            }
        } else {
            throw new IllegalArgumentException("Invalid key");
        }
    }
}