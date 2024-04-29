package space.tudsat.sam;

import java.util.Timer;
import java.util.TimerTask;
import java.util.Arrays;
import java.util.List;
import java.io.IOException;
import android.util.Log;
import android.app.NativeActivity;
import android.content.Intent;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Looper;
import android.location.Location;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.tasks.OnSuccessListener;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;

public class MainActivity extends NativeActivity {
    private FusedLocationProviderClient fusedLocationClient;
    private LocationCallback locationCallback;
    private UsbSerialPort serialPort;

    static {
        System.loadLibrary("sam_android");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this);
        locationCallback = new LocationCallback() {
            @Override
            public void onLocationResult(LocationResult locationResult) {
                if (locationResult == null) {
                    return;
                }
                for (Location location : locationResult.getLocations()) {
                    processLocation(location);
                }
            }
        };

        new Timer().scheduleAtFixedRate(new TimerTask() {
            @Override
            public void run() {
                processSerialPort();
            }
        }, 0, 10);

        getCurrentLocation();
        notifyOnNewIntent();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);

        notifyOnNewIntent();
    }

    @Override
    protected void onResume() {
        super.onResume();
        subscribeToLocation();
    }

    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        if (requestCode == 1 && grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            getCurrentLocation();
        }
    }

    private void openSerialPort() {
        if (serialPort != null) {
            return;
        }

        // Find all available drivers from attached devices.
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        if (availableDrivers.isEmpty()) {
            return;
        }

        // Open a connection to the first available driver.
        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
        if (connection == null) {
            // add UsbManager.requestPermission(driver.getDevice(), ..) handling here
            return;
        }

        serialPort = driver.getPorts().get(0); // Most devices have just one port (port 0)
        try {
            serialPort.open(connection);
            serialPort.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            notifyOnNewUsbConnectionStatus(true);
        } catch ( IOException e ) {
            Log.e("sam", "open failed");
            serialPort = null;
            notifyOnNewUsbConnectionStatus(false);
        }
    }

    private void processSerialPort() {
        openSerialPort();

        if (serialPort == null) {
            return;
        }

        try {
            byte[] readBuf = new byte[1024];
            int len = serialPort.read(readBuf, 5);
            notifyOnNewUsbData(readBuf, len);
        } catch ( IOException e ) {
            Log.e("sam", "IOException");
            serialPort = null;
            notifyOnNewUsbConnectionStatus(false);
        }

        try {
            byte[] uplinkData = checkForUplinkData();
            if (uplinkData != null) {
                serialPort.write(uplinkData, 3);
            }
        } catch ( IOException e ) {
            Log.e("sam", "IOException");
        }
    }

    private void getCurrentLocation() {
        if (checkSelfPermission("android.permission.ACCESS_FINE_LOCATION") != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[] {"android.permission.ACCESS_FINE_LOCATION"}, 1);
            return;
        }

        fusedLocationClient.getCurrentLocation(100, null)
            .addOnSuccessListener(this, new OnSuccessListener<Location>() {
                @Override
                public void onSuccess(Location location) {
                    // Got last known location. In some rare situations this can be null.
                    if (location != null) {
                        processLocation(location);
                    }
                }
            });

        subscribeToLocation();
    }

    private void subscribeToLocation() {
        if (checkSelfPermission("android.permission.ACCESS_FINE_LOCATION") != PackageManager.PERMISSION_GRANTED) {
            return;
        }

        LocationRequest locationRequest = new LocationRequest.Builder(10000)
            .setMinUpdateIntervalMillis(5000)
            .setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY)
            .build();
        fusedLocationClient.requestLocationUpdates(locationRequest,
            locationCallback,
            Looper.getMainLooper());
    }

    private void processLocation(Location location) {
        double lat = location.getLatitude();
        double lng = location.getLongitude();
        double alt = location.getAltitude();
        double acc = location.getAccuracy();
        notifyOnNewLocation(lat, lng, alt, acc);
    }

    private native void notifyOnNewIntent();
    private native void notifyOnNewLocation(double lat, double lng, double alt, double acc);
    private native void notifyOnNewUsbData(byte[] bytes, int len);
    private native void notifyOnNewUsbConnectionStatus(boolean connected);
    private native byte[] checkForUplinkData();
}
