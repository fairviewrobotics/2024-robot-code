package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableUtils {
    private NetworkTable table;

    /**
     * This class is a utility class for interfacing with network tables

     * @param table The string ID for a network table

     */
    public NetworkTableUtils(String table) {
        this.table = NetworkTableInstance.getDefault().getTable(table);
    }

    /**
     * This function returns the table of this instance of NetworkTableUtils
     * @return Returns the table as a NetworkTable
     */
    public NetworkTable getTable() {
        return this.table;
    }

    /**
     * This function gets a value from network tables as a double
     * @param key The key in Network Tables for the value
     * @param defaultValue If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a double
     */
    public double getDouble(String key, double defaultValue) {
        return this.table.getEntry(key).getDouble(defaultValue);
    }

    /**
     * This function gets a value from network tables as a String
     * @param key The key in Network Tables for the value
     * @param defaultValue If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a String
     */
    public String getString(String key, String defaultValue) {
        return this.table.getEntry(key).getString(defaultValue);
    }

    /**
     * This function gets a value from Network Tables as a double array
     * @param key The key in Network Tables for the value
     * @param doubles If the entry is not found or null we will return this
     * @return Returns the Network Table entry as a double array (double[])
     */
    public double[] getDoubleArray(String key, double[] doubles) {
        return this.table.getEntry(key).getDoubleArray(doubles);
    }

    public void  setDoubleArray(String key, double[] value) {
        this.table.getEntry(key).setDoubleArray(value);
    }

    /**
     * This function sets a double in network tables
     * @param key The key in Network Tables for the value
     * @param value What we are setting the entry to
     */
    public void setDouble(String key, double value) {
        this.table.getEntry(key).setDouble(value);
    }

    /**
     * This function sets a String in network tables
     * @param key The key in Network Tables for the value
     * @param value What we are setting the entry to
     */
    public void setString(String key, String value) {
        this.table.getEntry(key).setString(value);
    }

    /**
     * This function returns a entry as whatever it is
     * @param key The key in Network Tables for the value
     * @param defaultValue We will return this is the entry is invalid
     * @return Returns the entry as whatever it is
     */
    public Object getEntry(String key, Object defaultValue) {
        if (this.table.getEntry(key).exists()) {
            if (defaultValue instanceof Double) {
                return getDouble(key, (Double) defaultValue);
            } else if (defaultValue instanceof String) {
                return getString(key, (String) defaultValue);
            } else  if (defaultValue instanceof double[]) {
                return getDoubleArray(key, (double[]) defaultValue);
            }
        }
        return null;
    }

    /**
     * Sets a entry in Network Tables to a value
     * @param key The key in Network Tables for the value
     * @param value The value we are setting the entry to.
     */
        public void setEntry(String key, Object value) {
            if (this.table.getEntry(key).exists()) {
                if (value instanceof Double) {
                    setDouble(key, (Double) value);
                } else if (value instanceof String) {
                    setString(key, (String) value);
                } else  if (value instanceof double[]) {
                    setDoubleArray(key, (double[]) value);
                }
            }
        }
}