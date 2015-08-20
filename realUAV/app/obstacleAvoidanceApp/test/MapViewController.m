//
//  MapViewController.m
//  test
//
//  Created by Angelos Plastropoulos on 20/06/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "MapViewController.h"
#import "stdlib.h"
#import "Measurement.h"
#import "MyCircle.h"
#import "MapSettingsViewController.h"

#define ARC4RANDOM_MAX 0x100000000000
#define METERS_PER_MILE 1609.344

@interface MapViewController () <MKMapViewDelegate, MapSettingsViewControllerDelegate>

@property (weak, nonatomic) IBOutlet MKMapView *mapView;
@property (weak, nonatomic) IBOutlet UIImageView *gpsImView;
@property (weak, nonatomic) IBOutlet UIImageView *bluetoothImView;
@property (weak, nonatomic) IBOutlet UIImageView *radiationImView;
@property (weak, nonatomic) IBOutlet UIImageView *mappingMthdImView;

@property (nonatomic, weak) IBOutlet UILabel *lblCPS;
@property (nonatomic, weak) IBOutlet UILabel *lblCPSmax;

@end

@implementation MapViewController

bool gpsFix2, gpsUpdateReceiver2, bluetoothStatus;
Measurement *myMeasurement;
MKCoordinateRegion viewRegion;

// Two union structs to support the primary data types
// assemblage from the raw bytes sent from BLE
union U8f {
    uint8_t byte[4];
    float f;
};

union U4i {
    uint8_t byte[2];
    uint16_t i;
};

#pragma mark - Support internal functionality

- (void)viewDidLoad {
    [super viewDidLoad];
    
    NSLog(@"map view did load, map mode %d", _mappingMthd.intValue);
    
    myMeasurement = [Measurement new];
    
    // init max colour range to 50
    _maxRange = 50;
    _lblCPSmax.text = [NSString stringWithFormat:@"%d", _maxRange];
    
    // handle data transitions in the reception of data
    // between the two controllers
    gpsFix2 = true;
    gpsUpdateReceiver2 = true;
    bluetoothStatus = true;
    
    // declare delegates
    _ble.delegate = self;
    _mapView.delegate = self;
    
    Measurement *lastMeasurement = [_myArrayOfMeasurements lastObject];

    // set the zoom level of the map
    viewRegion = [self setMapZoomWith:lastMeasurement];
    [self loadMappingMthdImage];
    [self drawAllMeasuremets];
    [_mapView setRegion:viewRegion animated:YES];
    
    // init the status icons
    _gpsImView.image = [UIImage imageNamed:@"gpsFail"];
    _bluetoothImView.image = [UIImage imageNamed:@"bluetoothFail"];
    _radiationImView.image = [UIImage imageNamed:@"radioOK"];
    
    // animate the heart beat effect of the radiation icon
    CABasicAnimation *theAnimation;
    
    theAnimation=[CABasicAnimation animationWithKeyPath:@"opacity"];
    theAnimation.duration=1.0;
    theAnimation.repeatCount=HUGE_VALF;
    theAnimation.autoreverses=YES;
    theAnimation.fromValue=[NSNumber numberWithFloat:1.0];
    theAnimation.toValue=[NSNumber numberWithFloat:0.0];
    [_radiationImView.layer addAnimation:theAnimation forKey:@"animateOpacity"];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    // navigation between UIViewControllers
    if ([segue.identifier isEqualToString:@"showMapSettings"]) {
        UINavigationController *navigationController = segue.destinationViewController;
        MapSettingsViewController *mapSettingsViewController = [navigationController viewControllers][0];
        mapSettingsViewController.delegate = self;
        mapSettingsViewController.maxRange = _maxRange;
        mapSettingsViewController.mappingMthd =_mappingMthd.intValue;
        
    }
}

- (void)viewWillDisappear:(BOOL)animated {
    // disable the data reception
    NSLog(@"I'm going to dissappear");
    gpsUpdateReceiver2 = false;
}

- (void)viewWillAppear:(BOOL)animated {
    // enable the data reception
    NSLog(@"I'm going to appear");
    gpsUpdateReceiver2 = true;
    _ble.delegate = self;
    
    //Measurement *lastMeasurement = [_myArrayOfMeasurements lastObject];
    //viewRegion = [self setMapZoomWith:lastMeasurement];
    //[_mapView setRegion:viewRegion animated:YES];
}


#pragma mark - Actions & Delegate Actions

- (IBAction)cancel:(id)sender {
    [self.delegate mapViewControllerDidCancel:self withStatus:bluetoothStatus];
}

- (void)mapSettingsViewControllerDidCancel: (MapSettingsViewController *)controller {
    [self dismissViewControllerAnimated:YES completion:nil];
}

- (void)mapSettingsViewControllerDidSave:(MapSettingsViewController *)controller withCPSmax:(unsigned int)value withMapMthd:(unsigned int)mthdValue {
    
    // update the summary section according to the user's selctions made on the
    // relevant ViewController
    _maxRange = value;
    _lblCPSmax.text = [NSString stringWithFormat:@"%d", _maxRange];
    
    _mappingMthd = [NSNumber numberWithInt:mthdValue];
    
    Measurement *firstMeasurement = [_myArrayOfMeasurements firstObject];
    
    // draw again the measurements with the selected zoom level
    viewRegion = [self setMapZoomWith:firstMeasurement];
    [_mapView setRegion:viewRegion animated:YES];
    [_mapView removeOverlays:_mapView.overlays];
    [self drawAllMeasuremets];
    
    [self loadMappingMthdImage];
    //NSLog(@"The returned value is: %d", _maxRange);
    [self drawAllMeasuremets];
    [self dismissViewControllerAnimated:YES completion:nil];
}

- (IBAction)clear:(id)sender {
    // handle the actionSheets options
    // according to the user's choise
    
    UIAlertController *actionSheetView = [UIAlertController alertControllerWithTitle:@"Map actions"
                                                                  message:@"Please select one of the following actions:"
                                                           preferredStyle:UIAlertControllerStyleActionSheet];
    
    UIAlertAction *configMap = [UIAlertAction actionWithTitle:@"map settings"
                                                       style:UIAlertActionStyleDefault
                                                     handler:^(UIAlertAction *action) {
                                                         [self performSegueWithIdentifier:@"showMapSettings" sender:sender];
                                                         //[_mapView removeOverlays:_mapView.overlays];
                                                         [actionSheetView dismissViewControllerAnimated:YES completion:nil];
                                                     }];
    
    UIAlertAction *clearMap = [UIAlertAction actionWithTitle:@"clear the map"
                                                       style:UIAlertActionStyleDefault
                                                     handler:^(UIAlertAction *action) {
                                                         [_mapView removeOverlays:_mapView.overlays];
                                                         [actionSheetView dismissViewControllerAnimated:YES completion:nil];
                                                     }];
    
    UIAlertAction *cancel = [UIAlertAction actionWithTitle:@"cancel"
                                                     style:UIAlertActionStyleDefault
                                                   handler:^(UIAlertAction *action) {
                                                       [actionSheetView dismissViewControllerAnimated:YES completion:nil];
                                                   }];
    
    UIAlertAction *redrawMap = [UIAlertAction actionWithTitle:@"redraw the map"
                                                        style:UIAlertActionStyleDefault
                                                      handler:^(UIAlertAction *action) {
                                                          [_mapView removeOverlays:_mapView.overlays];
                                                          [self drawAllMeasuremets];
                                                          [actionSheetView dismissViewControllerAnimated:YES completion:nil];
                                                      }];
    
    
    [actionSheetView addAction:configMap];
    [actionSheetView addAction:redrawMap];
    [actionSheetView addAction:clearMap];
    [actionSheetView addAction:cancel];
    
    [self presentViewController:actionSheetView animated:YES completion:nil];
    
}

#pragma mark - BLE delegate to manage the connection (a subset of the available methods since the connection has already made)

-(void) bleDidUpdateRSSI:(NSNumber *) rssi {
    // When RSSI is changed, this will be called
    // display the value to the screen
    //_lblRSSI.text = rssi.stringValue;
}

-(void) readRSSITimer:(NSTimer *)timer {
    [_ble readRSSI];
}

- (void)bleDidDisconnect {
    // when BLE disconnected update the summary section
    bluetoothStatus = false;
    _bluetoothImView.image = [UIImage imageNamed:@"bluetoothFail"];
    NSLog(@"->Disconnected");
}


-(void) bleDidReceiveData:(unsigned char *)data length:(int)length {
    // When data is comming this method is called
    
    //[self updateStatusWith:@"In connection"];
    // update the bluetooth icon
    _bluetoothImView.image = [UIImage imageNamed:@"bluetoothOK"];
    
    if (gpsUpdateReceiver2) {
        //NSLog(@"Data is coming to map");
        //NSLog(@"Length: %d", length);
        
        // parse data, the message payload is 11 bytes
        for (int i = 0; i < length; i+=11)
        {
            //NSLog(@"0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X",
            //      data[i], data[i+1], data[i+2], data[i+3], data[i+4], data[i+5], data[i+6], data[i+7], data[i+8], data[i+9], data[i+10]);
            
            // intersted only if the connection is alive
            char fixOK = data[i] & 0x01;
            
            if (fixOK == 0x00)
            {
                NSLog(@"GPS has not locked\n");
                gpsFix2 = false;
                _gpsImView.image = [UIImage imageNamed:@"gpsFail"];
                
                //_lblGPSState.text = @"Not locked";
                
            } else if (fixOK == 0x01) {
                NSLog(@"GPS has locked\n");
                gpsFix2 = true;
                // update the GPS icon
                _gpsImView.image = [UIImage imageNamed:@"gpsOK"];
                
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
                
                CLLocationCoordinate2D location;
                location.latitude = latValue;
                location.longitude = lonValue;
                
                // add date to the Measurement object
                myMeasurement.location = location;
                myMeasurement.cps = cpsValue;
                
                // and then to the array
                [_myArrayOfMeasurements addObject:myMeasurement];
                
                // update GUI
                _lblCPS.text = [NSString stringWithFormat:@"%lu", (unsigned long)cpsValue];
                
                //NSLog(@"lat: %f lon: %f cps: %d\n", myMeasurement.location.latitude, myMeasurement.location.longitude, myMeasurement.cps);
                [self drawMeasurement:myMeasurement];
                
            }
        }
    }
}

#pragma mark - Helper methods

- (MKCoordinateRegion)setMapZoomWith:(Measurement *)myMeasurement {
    // set the Map zoom level according to the mapping method selected
    
    MKCoordinateRegion viewRegion;
    
    if (_mappingMthd.intValue == 0) {
       viewRegion  = MKCoordinateRegionMakeWithDistance(myMeasurement.location, 0.5*METERS_PER_MILE, 0.5*METERS_PER_MILE);
        //NSLog(@"Lat:%f Lon:%f, mode:%d\n", myMeasurement.location.latitude, myMeasurement.location.longitude, _mappingMthd.intValue);
        _circleRadius = 10;
    } else if (_mappingMthd.intValue == 1) {
        viewRegion  = MKCoordinateRegionMakeWithDistance(myMeasurement.location, 1.5*METERS_PER_MILE, 1.5*METERS_PER_MILE);
        //NSLog(@"Lat:%f Lon:%f, mode:%d\n", myMeasurement.location.latitude, myMeasurement.location.longitude, _mappingMthd.intValue);
        _circleRadius = 40;
    } else if (_mappingMthd.intValue == 2) {
        viewRegion  = MKCoordinateRegionMakeWithDistance(myMeasurement.location, 1.0*METERS_PER_MILE, 1.0*METERS_PER_MILE);
        //NSLog(@"Lat:%f Lon:%f, mode:%d\n", myMeasurement.location.latitude, myMeasurement.location.longitude, _mappingMthd.intValue);
        _circleRadius = 20;
    }
    
    return viewRegion;
}

- (void)loadMappingMthdImage {
    // update the icon with the appropriate image
    
    if (_mappingMthd.intValue == 0) {
        _mappingMthdImView.image = [UIImage imageNamed:@"foot"];
    } else if (_mappingMthd.intValue == 1) {
        _mappingMthdImView.image = [UIImage imageNamed:@"car"];
    } else if (_mappingMthd.intValue == 2) {
        _mappingMthdImView.image = [UIImage imageNamed:@"uav"];
    }
}


- (void)drawAllMeasuremets {
    // draw all measurements of the NSAray
    int count;
    count = [_myArrayOfMeasurements count];
    
    for (int i = 0; i<count; i++) {
        [self drawMeasurement:[_myArrayOfMeasurements objectAtIndex:i]];
    }
}

- (void)drawMeasurement:(Measurement *)measerument {
    // draw a sigle measurement object in the map
    
    // add randmness to mock the GPS face movement
    //float randomVal = ((float)arc4random() / ARC4RANDOM_MAX);
    //NSLog(@"random num: %f\n", randomVal);
    
    //CLLocationCoordinate2D center = {measerument.location.latitude+randomVal, measerument.location.longitude};
    CLLocationCoordinate2D center = {measerument.location.latitude, measerument.location.longitude};
    MyCircle *circle = [MyCircle circleWithCenterCoordinate:center radius:_circleRadius];
    circle.cpsValue = myMeasurement.cps;
    
    [self.mapView addOverlay:circle];
}

- (UIColor *)chooseColor:(unsigned) cpsValue {
    // select the measurement colour according to the cps value
    
    UIColor *selectedColor;
    // define the unit according to the maximum selected range
    float unit = _maxRange/7.0;
    
    if (cpsValue <= unit) {
        selectedColor = [UIColor purpleColor];
    } else if (cpsValue > unit && cpsValue <= 2*unit) {
        selectedColor = [UIColor blueColor];
    } else if (cpsValue > 2*unit && cpsValue <= 3*unit) {
        selectedColor = [UIColor cyanColor];
    } else if (cpsValue > 3*unit && cpsValue <= 4*unit) {
        selectedColor = [UIColor greenColor];
    } else if (cpsValue > 4*unit && cpsValue <= 5*unit) {
        selectedColor = [UIColor yellowColor];
    } else if (cpsValue > 5*unit && cpsValue <= 6*unit) {
        selectedColor = [UIColor orangeColor];
    } else if (cpsValue > 6*unit && cpsValue <= 7*unit) {
        selectedColor = [UIColor redColor];
    }
    
    return selectedColor;
}

#pragma mark - Map view delegates

- (MKOverlayView *)mapView:(MKMapView *)mapView viewForOverlay:(id<MKOverlay>)overlay {
    
    // handle the MyCircle custom overlay which is a custom MK circle
    MKCircleView *circleView = [[MKCircleView alloc] initWithOverlay:overlay];
    
    MyCircle *myCircle = (MyCircle *)overlay;
    
    //NSLog(@"My overlay info: %d\n", myCircle.cpsValue);
    [circleView setFillColor: [self chooseColor: myCircle.cpsValue]];
    //[circleView setFillColor:[UIColor redColor]];
    //[circleView setStrokeColor:[UIColor blackColor]];
    [circleView setAlpha:1.0f];
    return circleView;
}

@end
