package com.example.gyroscopeapp;

import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;


import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;

import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttMessage;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;

import info.mqtt.android.service.Ack;
import info.mqtt.android.service.MqttAndroidClient;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    static String mqttBroker = "tcp://broker.hivemq.com:1883";
    static String username = "";
    static String password = "";
    MqttAndroidClient client;

    private SensorManager sensorManager;
    private TextView angleText;
    private Button calibrationBtn;
    private double pitchGyr;
    private double q;
    private double pitchAcc;
    private double offset = 0.0;
    private ArrayList<Double> calibrationSamples;
    private boolean connected = false;
    /*private double phiHat;
    private double thetaHat_acc;
    private double alpha = 0.05;*/

    private double xHat[];
    private double xHatBar[];
    private double P[][];
    private double PBar[][];
    private double Q[][];
    private double K[];
    private double dt;
    private double r;

    long SAMPLE_TIME = 100; //milliseconds
    long lastMillisGyr = 0;
    long lastMillisAcc = 0;
    long calibrationCounter;
    long calibrationTimer = 5000;

    private TextView accText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        angleText = findViewById(R.id.angleText);
        accText = findViewById(R.id.accText);
        calibrationBtn = findViewById(R.id.calibrateBtn);
        initializeSensor();
        connectBroker();
        initializeFilter();
    }

    private void initializeSensor(){
        pitchGyr = 0.0;
        //phiHat = 0.0;
        calibrationSamples = new ArrayList<>();
        sensorManager =(SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_GAME,
                SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_GAME,
                SensorManager.SENSOR_DELAY_GAME);
    }

    public void publish(String payload){
        String topic = "pitch/degrees";
        MqttMessage message = new MqttMessage(payload.getBytes());
        message.setQos(1);
        message.setRetained(true);
        if(connected == true) {
            client.publish(topic, message);
        }else{
            accText.setText("connecting...");
        }
    }

    public void subscribe(){
        String topic = "pitch/degrees";
        int qos = 1;
        client.subscribe(topic, qos);
    }

    public void connectBroker(){
        String clientId = MqttClient.generateClientId();
        client = new MqttAndroidClient(this.getApplicationContext(), mqttBroker, clientId, Ack.AUTO_ACK);
        MqttConnectOptions options = new MqttConnectOptions();
        options.setUserName(username);
        options.setPassword(password.toCharArray());
        IMqttToken token = client.connect();
        token.setActionCallback(new IMqttActionListener() {
            @Override
            public void onSuccess(IMqttToken asyncActionToken) {
                Toast toast = Toast.makeText(MainActivity.this, "connected to server", Toast.LENGTH_SHORT);
                toast.show();
                connected = true;
                subscribe();
            }

            @Override
            public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                Toast toast = Toast.makeText(MainActivity.this, "failed to connect to server", Toast.LENGTH_SHORT);
                toast.show();
            }
        });

        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                Toast toast = Toast.makeText(MainActivity.this, "connection lost", Toast.LENGTH_SHORT);
                toast.show();
                connected = false;
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                if (topic.matches("pitch/degrees")){
                    String received = new String(message.getPayload());

                    accText.setText(received + "º");
                }
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {

            }
        });

    }


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent != null){
            if (sensorEvent.sensor != null){
                long millis = System.currentTimeMillis() / SAMPLE_TIME;

                if (sensorEvent.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                    if (millis != lastMillisGyr) {
                        /*double p = sensorEvent.values[0];
                        double qq = sensorEvent.values[1];
                        double rr = sensorEvent.values[2];*/

                        q = sensorEvent.values[0];

                        //double phiDot = p + Math.tan(pitchGyr) * (Math.sin(phiHat) * qq + Math.cos(phiHat) * rr);
                        //double thetaDot = Math.cos(phiHat) * qq - Math.sin(phiHat) * rr;

                        //phiHat = phiHat + (SAMPLE_TIME / 1000.0) * phiDot;
                        pitchGyr = pitchGyr + ((SAMPLE_TIME / 1000.0) * q - offset);


                        /*int pitchAccInDegrees = (int) Math.round(phiHat * 180 / Math.PI);

                        accText.setText(pitchAccInDegrees + "º");*/

                        lastMillisGyr = millis;
                    }
                }
                if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                    if (millis != lastMillisAcc) {
                        double accX = sensorEvent.values[0];
                        double accY = sensorEvent.values[1];
                        double accZ = sensorEvent.values[2];
                        pitchAcc = Math.atan2(accY, accZ) - offset;

                        /*int pitchAccInDegrees = (int) Math.round(pitchAcc * 180 / Math.PI);

                        accText.setText(pitchAccInDegrees + "º");*/

                        //thetaHat_acc = Math.asin(accX/9.8);

                        kalmanFilter();

                        if(calibrationCounter > 0){
                            calibrationSamples.add(xHat[0]);
                            calibrationCounter = calibrationCounter - SAMPLE_TIME;
                            if (calibrationCounter <= 0){
                                offset = offset + calibrationSamples.stream().mapToDouble(val -> val).average().orElse(0.0);
                                calibrationSamples.removeAll(calibrationSamples);
                                Toast toast = Toast.makeText(this, "Calibration complete", Toast.LENGTH_SHORT);
                                toast.show();
                                calibrationBtn.setText("calibrate");
                            }
                        }

                        int pitchInDegrees = (int) Math.round(xHat[0] * 180 / Math.PI);
                        angleText.setText(pitchInDegrees + "º");

                        publish(Integer.toString(pitchInDegrees));

                        lastMillisAcc = millis;
                    }
                }
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
    }

    public void initializeFilter(){
        xHat = new double[] {0.0, 0.0};
        xHatBar = new double[] {0.0, 0.0};
        P = new double[][] {{0.1, 0.1},
                            {0.1, 0.1}};
        PBar = new double[][] {{0.1, 0.1},
                                {0.1, 0.1}};
        Q = new double[][] {{0.01, 0.01},
                            {0.01, 0.01}};
        K = new double[] {0.0, 0.0};
        dt = SAMPLE_TIME/1000.0;
        r = 0.1;
    }

    public void kalmanFilter(){
        xHatBar[0] = xHat[0] + dt * (q - xHat[1]);
        xHatBar[1] = xHat[1];
        PBar[0][0] = P[0][0] + dt * ((dt * P[1][1]) - P[1][0] - P[0][1] + Q[0][0]);
        PBar[0][1] = P[0][1] + dt * (Q[0][1] - P[1][1]);
        PBar[1][0] = P[1][0] + dt * (Q[1][0] - P[1][1]);
        PBar[1][1] = P[1][1] + dt * (Q[1][1]);

        K[0] = PBar[0][0] / (PBar[0][0] + r);
        K[1] = PBar[1][0] / (PBar[0][0] + r);
        xHat[0] = xHatBar[0] + K[0] * (pitchAcc - xHatBar[0]);
        xHat[1] = xHatBar[1] + K[1] * (pitchAcc - xHatBar[0]);
        P[0][0] = PBar[0][0] - (K[0] * PBar[0][0]);
        P[0][1] = PBar[0][1] - (K[0] * PBar[0][1]);
        P[1][0] = PBar[1][0] - (K[1] * PBar[0][0]);
        P[1][1] = PBar[1][1] - (K[1] * PBar[0][1]);
    }

    public void calibrate(View view){
        calibrationCounter = calibrationTimer;
        calibrationBtn.setText("calibrating...");
    }

    @Override
    public void onDestroy() {
        sensorManager.unregisterListener(this);
        client.disconnect();
        super.onDestroy();
    }
}