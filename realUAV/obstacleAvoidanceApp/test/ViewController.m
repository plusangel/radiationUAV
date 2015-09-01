//
//  ViewController.m
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
@property (nonatomic, weak) IBOutlet UILabel *lbldist0;
@property (nonatomic, weak) IBOutlet UILabel *lbldist90;
@property (nonatomic, weak) IBOutlet UILabel *lbldist180;

@property (nonatomic, weak) IBOutlet UIView *view0;
@property (nonatomic, weak) IBOutlet UIView *view90;
@property (nonatomic, weak) IBOutlet UIView *view180;

@property (nonatomic, weak) IBOutlet UIBarButtonItem *bbtnSettings;
@property (nonatomic, weak) IBOutlet UIBarButtonItem *bbtnConnectivity;
@property (nonatomic, weak) IBOutlet UIActivityIndicatorView *indConnection;

@property (weak, nonatomic) IBOutlet UISwitch *swtReverse;

@end

bool reverse;

@implementation ViewController

// Two union structs to support the primary data types
// assemblage from the raw bytes sent from BLE
union U8i {
    // Floaf32
    uint8_t byte[4];
    uint32_t i;
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
    
    // various initiliasations
    _fwdDistance = 100;
    _sdDistance = 100;
    reverse = false;
    
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


- (void)viewWillAppear:(BOOL)animated {
    // activate the BLE reception
    //NSLog(@"I'm going to appear");
    _ble.delegate = self;
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // navigation between UIViewControllers
    if ([segue.identifier isEqualToString:@"showSettings"]) {
        UINavigationController *navigationController = segue.destinationViewController;
        SettingsViewController *settingsViewController = [navigationController viewControllers][0];
        settingsViewController.delegate = self;
        settingsViewController.fwdDist = _fwdDistance;
        settingsViewController.sdDist = _sdDistance;
    }
}

#pragma mark - Delegate method for the SettingsViewController

- (void)settingsViewControllerDidCancel:(SettingsViewController *)controller {
    [self dismissViewControllerAnimated:YES completion:nil];
}

- (void)settingsViewControllerDidSave:(SettingsViewController *)controller withFWdist:(unsigned int)fwdValue withSDdist:(unsigned int)sdValue {
    _fwdDistance = fwdValue;
    _sdDistance = sdValue;
    
    [self dismissViewControllerAnimated:YES completion:nil];
}

#pragma mark - Actions

- (IBAction)switchChangeValue:(id)sender {
    // enable the reverse mode if the UISwitch in On
    if ([sender isOn]) {
        reverse = true;
    } else {
        reverse = false;
    }}

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
    _lblRSSI.text= @"----";
    
    NSLog(@"->Disconnected");
    
    _bbtnConnectivity.title = @"connect";
    [_indConnection stopAnimating];
    
    [self resetLabels];
    
    [rssiTimer invalidate];
}

-(void) bleDidUpdateRSSI:(NSNumber *) rssi {
    // When RSSI is changed, this will be called
    // display the value to the screen
    _lblRSSI.text = [NSString stringWithFormat:@"%d dB", rssi.intValue];
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
    
    // Schedule to read RSSI every 1 sec.
    rssiTimer = [NSTimer scheduledTimerWithTimeInterval:(float)1.0 target:self selector:@selector(readRSSITimer:) userInfo:nil repeats:YES];
}

-(void) bleDidReceiveData:(unsigned char *)data length:(int)length {
    // When data is comming this method is called
    
    [self updateStatusWith:@"In connection"];
    
    NSLog(@"Length: %d", length);
    
    // parse data, the message payload is 11 bytes
    for (int i = 0; i < length; i+=6)
    {
        NSLog(@"0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X",
              data[i], data[i+1], data[i+2], data[i+3], data[i+4], data[i+5]);
        
        union U4i dir;
        union U8i dist;
        dist.byte[0] = data[i+3];
        dist.byte[1] = data[i+2];
        dist.byte[2] = data[i+1];
        dist.byte[3] = data[i];
        
        dir.byte[0] = data[i+5];
        dir.byte[1] = data[i+4];
        
        int16_t direction = dir.i;
        int32_t distance = dist.i;
        
        NSLog(@"%d: %d\n", direction, distance);
        
        // update GUI
        if (direction == 0) {
            if (reverse == false) {
                [self displayDist:distance inDirect:direction];
            } else if (reverse == true) {
                [self displayDist:distance inDirect:direction+180];
            }
        } else if (direction == 90) {
            [self displayDist:distance inDirect:direction];
        } else if (direction == 180){
            if (reverse == false) {
                [self displayDist:distance inDirect:direction];
            } else if (reverse == true) {
                [self displayDist:distance inDirect:direction-180];
            }
        }
    }
}

#pragma mark - Helper methods

-(void)displayDist:(int32_t)myDist inDirect:(int16_t)myDir {
    // This finction updates the colour of the regions around the UAV
    // by comparing the distance thresholds with measured distances
    // in the three predefined directions
    
    // When myDist == 0 means that the LIDAR-Lite measures more than 40m
    if (myDir == 0) {
        _lbldist0.text = [NSString stringWithFormat:@"%d cm", myDist];
        if ((myDist < _sdDistance) && (myDist != 0)) {
            _view0.backgroundColor = [UIColor redColor];
        } else {
            _view0.backgroundColor = [UIColor greenColor];
        }
    } else if (myDir == 90) {
        _lbldist90.text = [NSString stringWithFormat:@"%d cm", myDist];
        if ((myDist < _fwdDistance) && (myDist != 0)) {
            _view90.backgroundColor = [UIColor redColor];
        } else {
            _view90.backgroundColor = [UIColor greenColor];
        }
    } else if (myDir == 180) {
        _lbldist180.text = [NSString stringWithFormat:@"%d cm", myDist];
        if ((myDist < _sdDistance) && (myDist != 0)) {
            _view180.backgroundColor = [UIColor redColor];
        } else {
            _view180.backgroundColor = [UIColor greenColor];
        }
    }
}

-(void)resetLabels {
    // reset all labels
    _lbldist0.text = @"----";
    _lbldist90.text = @"----";
    _lbldist180.text = @"----";
}

-(void)updateStatusWith:(NSString *)status {
    // update the information board
    [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(updateStatusWith:) object:@""];
    _lblStatusUpdate.text = status;
    [self performSelector:@selector(updateStatusWith:) withObject:@"" afterDelay:5];
}

@end
