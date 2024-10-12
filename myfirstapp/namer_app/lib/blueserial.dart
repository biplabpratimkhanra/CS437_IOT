import 'dart:async';
import 'dart:typed_data';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'dart:convert' show utf8;

class Blueserial {
  BluetoothConnection? _connection;
  bool isConnecting = false; // To track the connection status
  StreamController<String> _receivedDataController = StreamController<String>.broadcast();

  // Connect to the device
  Future<void> connectToDevice() async {
    // Check if already connected
    if (_connection != null && _connection!.isConnected) {
      print('Already connected to the device.');
      return;
    }

    // Check if the connection process is already in progress
    if (isConnecting) {
      print('Connection is already in progress.');
      return;
    }

    isConnecting = true; // Set to true while connecting

    List<BluetoothDevice> devices = await FlutterBluetoothSerial.instance.getBondedDevices();
    String deviceAddress = 'D8:3A:DD:9C:81:93'; // Replace with your device's address

    BluetoothDevice device = devices.firstWhere((d) => d.address == deviceAddress);

    try {
      // Establish connection
      _connection = await BluetoothConnection.toAddress(device.address);
      print('Connected to ${device.name}');
     
      // Start listening for incoming data
      _connection!.input?.listen((Uint8List recvd) {
        String receivedData = utf8.decode(recvd);
        // print('Received data: $receivedData');
       
        // Add received data to the stream controller
        _receivedDataController.add(receivedData);
      }).onDone(() {
        print('Connection closed.');
        _handleConnectionClosure();
      });

    } catch (error) {
      print('Error connecting to device: $error');
      isConnecting = false;
    } finally {
      isConnecting = false; // Reset after connection attempt
    }
  }

  // Method to send data to the device
  Future<void> sendData(String data) async {
    if (_connection != null && _connection!.isConnected) {
      _connection!.output.add(utf8.encode(data));
      await _connection!.output.allSent;
      print('Data sent: $data');
    } else {
      print('No active connection. Cannot send data.');
    }
  }

  // Method to receive data as a stream (for continuous listening)
  Stream<String> get onDataReceived => _receivedDataController.stream;

  // Method to disconnect from the device
  Future<void> disconnect() async {
    if (_connection != null && _connection!.isConnected) {
      await _connection!.close();
      _handleConnectionClosure();
      print('Disconnected from device.');
    } else {
      print('No active connection to disconnect.');
    }
  }

  // Method to handle connection closure
  void _handleConnectionClosure() {
    _connection = null;
    isConnecting = false;
    print('Connection closed and reset.');
  }

  // Dispose of stream controller to avoid memory leaks
  void dispose() {
    _receivedDataController.close();
  }
}
