import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'dart:convert' show utf8;

class Blueserial {
  BluetoothConnection? _connection;
  bool isConnecting = false;
  StreamController<String> _receivedDataController = StreamController<String>.broadcast();

  // Connect to the device
  Future<void> connectToDevice() async {
    if (_connection != null && _connection!.isConnected) {
      print('Already connected to the device.');
      return;
    }

    if (isConnecting) {
      print('Connection is already in progress.');
      return;
    }

    isConnecting = true;

    List<BluetoothDevice> devices = await FlutterBluetoothSerial.instance.getBondedDevices();
    String deviceAddress = 'D8:3A:DD:9C:81:93'; // Replace with your device's address

    BluetoothDevice device = devices.firstWhere((d) => d.address == deviceAddress);

    try {
      _connection = await BluetoothConnection.toAddress(device.address);
      print('Connected to ${device.name}');

      // Start listening for incoming data
      _connection!.input?.listen((Uint8List recvd) {
        String receivedData = utf8.decode(recvd);
        print('Received data: $receivedData');
        _receivedDataController.add(receivedData); // Push data to stream controller
      }).onDone(() {
        print('Connection closed.');
        _handleConnectionClosure();
      });
    } catch (error) {
      print('Error connecting to device: $error');
      isConnecting = false;
    } finally {
      isConnecting = false;
    }
  }

  // Send data to the device
  Future<void> sendData(String data) async {
    if (_connection != null && _connection!.isConnected) {
      _connection!.output.add(utf8.encode(data));
      await _connection!.output.allSent;
      print('Data sent: $data');
    } else {
      print('No active connection. Cannot send data.');
    }
  }

  // Receive data as a stream
  Stream<String> get onDataReceived => _receivedDataController.stream;

  // Disconnect from the device
  Future<void> disconnect() async {
    if (_connection != null && _connection!.isConnected) {
      await _connection!.close();
      _handleConnectionClosure();
      print('Disconnected from device.');
    } else {
      print('No active connection to disconnect.');
    }
  }

  // Handle connection closure
  void _handleConnectionClosure() {
    _connection = null;
    isConnecting = false;
    print('Connection closed and reset.');
  }

  // Dispose stream controller
  void dispose() {
    _receivedDataController.close();
  }
}

// The Flutter App
class BluetoothApp extends StatefulWidget {
  @override
  _BluetoothAppState createState() => _BluetoothAppState();
}

class _BluetoothAppState extends State<BluetoothApp> {
  final Blueserial _blueSerial = Blueserial();
  String _receivedData = "No data received yet"; // To display received data

  @override
  void initState() {
    super.initState();

    // Listen to the Bluetooth data stream and update the state
    _blueSerial.onDataReceived.listen((data) {
      setState(() {
        _receivedData = data;
      });
    });
  }

  @override
  void dispose() {
    _blueSerial.dispose(); // Dispose of the Bluetooth stream when done
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Bluetooth Serial Communication'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(
              'Received Data:',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
            ),
            SizedBox(height: 20),
            Text(
              _receivedData,
              style: TextStyle(fontSize: 16, color: Colors.blue),
            ),
            SizedBox(height: 40),
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                ElevatedButton(
                  onPressed: () async {
                    await _blueSerial.connectToDevice();
                  },
                  child: Text('Connect'),
                ),
                SizedBox(width: 20),
                ElevatedButton(
                  onPressed: () async {
                    await _blueSerial.sendData("power level");
                  },
                  child: Text('power level'),
                ),
                SizedBox(width: 20),
                ElevatedButton(
                  onPressed: () async {
                    await _blueSerial.disconnect();
                  },
                  child: Text('Disconnect'),
                ),
              ],
            )
          ],
        ),
      ),
    );
  }
}

void main() {
  runApp(MaterialApp(
    home: BluetoothApp(),
  ));
}

