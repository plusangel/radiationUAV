//
//  ViewController.m
//  test
//
//  Created by Angelos Plastropoulos on 20/06/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "ViewController.h"
#import <MapKit/MapKit.h>
#import "Measurement.h"

@interface ViewController ()
@property (nonatomic, weak) IBOutlet UILabel *lblStatusUpdate;
@property (nonatomic, weak) IBOutlet UILabel *lblRSSI;
@property (nonatomic, weak) IBOutlet UILabel *lblGPSState;
@property (nonatomic, weak) IBOutlet UILabel *lblLat;
@property (nonatomic, weak) IBOutlet UILabel *lblLon;
@property (nonatomic, weak) IBOutlet UILabel *lblCps;
@property (nonatomic, weak) IBOutlet UILabel *lblCount;

@property (nonatomic, weak) IBOutlet UIBarButtonItem *bbtnMap;
@property (nonatomic, weak) IBOutlet UIBarButtonItem *bbtnConnectivity;
@property (nonatomic, weak) IBOutlet UIActivityIndicatorView *indConnection;
@end

@implementation ViewController

bool gpsFix, gpsUpdateReceiver;
int numOfSats;

// Two union structs to support the primary data types
// assemblage from the raw bytes sent from BLE
union U8f {
    // Floaf32
    uint8_t byte[4];
    float f;
};

union U4i {
    // Int16
    uint8_t byte[2];
    uint16_t i;
};

#pragma mark - Support internal functionality

- (void)viewDidLoad {
    [super viewDidLoad];
    
    [self updateStatusWith:@"Initializing"];
    
    // init the BLE manager
    _ble = [[BLE alloc] init];
    [_ble controlSetup];
    _ble.delegate = self;
    
    // init measurements array
    _myArrayMeasurements = [[NSMutableArray alloc] init];
    
    // various initiliasations
    gpsFix = false;
    gpsUpdateReceiver = true;
    _lblGPSState.text = @"not locked";
    _bbtnMap.enabled = false;
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewWillDisappear:(BOOL)animated {
    // deactivate the BLE reception
    //NSLog(@"I'm going to dissappear");
    gpsUpdateReceiver = false;
}

- (void)viewWillAppear:(BOOL)animated {
    // activate the BLE reception
    //NSLog(@"I'm going to appear");
    gpsUpdateReceiver = true;
    _ble.delegate = self;
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // navigation between UIViewControllers
    if ([segue.identifier isEqualToString:@"showMap"]) {
        UINavigationController *navigationController = segue.destinationViewController;
        MapViewController *mapViewController = [navigationController viewControllers][0];
        mapViewController.delegate = self;
        mapViewController.ble = _ble;
        mapViewController.myArrayOfMeasurements = _myArrayMeasurements;
    }
}

#pragma mark - Delegate method for the MapViewController

- (void)mapViewControllerDidCancel: (MapViewController *)controller withStatus:(BOOL)status {
    if (status) {
        NSLog(@"STATUS: Connected\n");
    } else {
        NSLog(@"STATUS: Disconnected\n");
        [self resetLabels];
    }

    [self dismissViewControllerAnimated:YES completion:nil];
}

#pragma mark - BLE delegate to manage the connection

NSTimer *rssiTimer;

- (void)bleDidDiscoverPeripheral {
    [self updateStatusWith:@"Found Bluetooth Device"];
}

-(void)bleNoPeripheralsFound {
    // If no compatible device is found try again after a short delay
    [self updateStatusWith:@"No Bluetooth Device Found will rescan in 5 seconds"];
    [self performSelector:@selector(btnScanForPeripherals:) withObject:self afterDelay:5];
}

- (IBAction)btnScanForPeripherals:(id)sender {
    // Connect button will call to this
    
    [self updateStatusWith:@"Scanning for bluetooth device"];
    
    // if it is already connected leave
    if (_ble.activePeripheral)
        if(_ble.activePeripheral.state == CBPeripheralStateConnected) {
            [[_ble CM] cancelPeripheralConnection:[_ble activePeripheral]];
            _bbtnConnectivity.title = @"connect";
            return;
        }
    
    if (_ble.peripherals)
        _ble.peripherals = nil;
    
    // find peripherals with timeout time equal to 5
    [_ble findBLEPeripherals:5];
    
    [NSTimer scheduledTimerWithTimeInterval:(float)2.0 target:self selector:@selector(connectionTimer:) userInfo:nil repeats:NO];
    
    // take care of GUI
    [_indConnection startAnimating];
    _bbtnConnectivity.enabled = NO;
}

-(void) connectionTimer:(NSTimer *)timer {
    // callback method of the NSTimer for connection
    
    _bbtnConnectivity.enabled = YES;
    _bbtnConnectivity.title = @"disconnect";
    
    if (_ble.peripherals.count > 0) {
        [_ble connectPeripheral:[_ble.peripherals objectAtIndex:0]];
    } else {
        [self updateStatusWith:@"No compatible device found"];
        _bbtnConnectivity.title = @"connect";
        [_indConnection stopAnimating];
    }
}

- (void)bleDidDisconnect {
    // housekeeping when disconnect occurs
    
    [self updateStatusWith:@"Disconnecting"];
    _bbtnMap.enabled = false;
    _lblRSSI.text= @"----";
    
    NSLog(@"->Disconnected");
    
    _bbtnConnectivity.title = @"connect";
    [_indConnection stopAnimating];
    
    _lblLat.enabled = false;
    _lblLon.enabled = false;
    
    [self resetLabels];
    
    [rssiTimer invalidate];
}

-(void) bleDidUpdateRSSI:(NSNumber *) rssi {
    // When RSSI is changed, this will be called
    // display the value to the screen
    _lblRSSI.text = rssi.stringValue;
}

-(void) readRSSITimer:(NSTimer *)timer {
    [_ble readRSSI];
}

-(void) bleDidConnect {
    // tasks to do when a connection established
    CBPeripheral *p = [_ble.peripherals objectAtIndex:0];
    
    // display the name of the ble device
    [self updateStatusWith:[NSString stringWithFormat:@"Connected to %@", p.name]];
    NSLog(@"->Connected");
    
    [_indConnection stopAnimating];
    _bbtnConnectivity.enabled = YES;
    _bbtnConnectivity.title = @"disconnect";
    
    _lblLat.enabled = true;
    _lblLon.enabled = true;
    
    // Schedule to read RSSI every 1 sec.
    rssiTimer = [NSTimer scheduledTimerWithTimeInterval:(float)1.0 target:self selector:@selector(readRSSITimer:) userInfo:nil repeats:YES];
}

-(void) bleDidReceiveData:(unsigned char *)data length:(int)length {
    // When data is comming this method is called
    
    [self updateStatusWith:@"In connection"];
    
    if (gpsUpdateReceiver) {
        
        NSLog(@"Length: %d", length);
        
        // parse data, the message payload is 11 bytes
        for (int i = 0; i < length; i+=11)
        {
            NSLog(@"0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X",
                  data[i], data[i+1], data[i+2], data[i+3], data[i+4], data[i+5], data[i+6], data[i+7], data[i+8], data[i+9], data[i+10]);
            
            // break the status byte to its components
            char fixOK = data[i] & 0x01;
            char sats = data[i] & 0x1E;
            sats = sats >> 1;
            //NSLog(@"Status bit 0x%02x\n", sats);
            //NSLog(@"Status bit %d\n", (int)sats);
            numOfSats = (int)sats;
            
            if (fixOK == 0x00)
            {
                NSLog(@"GPS has not locked\n");
                gpsFix = false;
                
                [self resetLabels];
                
            } else if (fixOK == 0x01) {
                NSLog(@"GPS has locked\n");
                gpsFix = true;
                _bbtnMap.enabled = true;
                
                if(numOfSats != 0) {
                    _lblGPSState.text = [NSString stringWithFormat:@"locked to %d sats", numOfSats];
                }
                
                // since the GPS is locked put together the bytes to form
                // the required floats and int
                union U8f lat, lon;
                lat.byte[0] = data[i+4];
                lat.byte[1] = data[i+3];
                lat.byte[2] = data[i+2];
                lat.byte[3] = data[i+1];
                
                lon.byte[0] = data[i+8];
                lon.byte[1] = data[i+7];
                lon.byte[2] = data[i+6];
                lon.byte[3] = data[i+5];
                
                union U4i cps;
                
                cps.byte[0] = data[i+10];
                cps.byte[1] = data[i+9];
                
                int cpsValue = cps.i;
                
                float latValue = lat.f;
                float lonValue = lon.f;
                
                NSLog(@"lat: %f lon: %f cps:%d\n", latValue, lonValue, cpsValue);
                
                // update GUI
                _lblLat.text = [NSString stringWithFormat:@"%f", latValue];
                _lblLon.text = [NSString stringWithFormat:@"%f", lonValue];
                _lblCps.text = [NSString stringWithFormat:@"%d", cpsValue];
                
                CLLocationCoordinate2D location;
                location.latitude = latValue;
                location.longitude = lonValue;
                
                // create the relevant object
                Measurement *myMeasurement = [Measurement new];
                myMeasurement.location = location;
                myMeasurement.cps = cpsValue;
                
                [_myArrayMeasurements addObject:myMeasurement];
                
                _lblCount.text = [NSString stringWithFormat:@"%lu", (unsigned long)[_myArrayMeasurements count]];
                
            }
        }
    }
}

#pragma mark - Helper methods

-(void)resetLabels {
    // reset all labels
    _lblGPSState.text = @"Not locked";
    _lblLat.text = @"----";
    _lblLon.text = @"----";
    _lblCps.text = @"----";
    _lblCount.text = @"----";
}

-(void)updateStatusWith:(NSString *)status {
    // update the information board
    [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(updateStatusWith:) object:@""];
    _lblStatusUpdate.text = status;
    [self performSelector:@selector(updateStatusWith:) withObject:@"" afterDelay:5];
}

@end
