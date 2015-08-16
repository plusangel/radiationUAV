//
//  ViewController.m
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 05/07/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#include <math.h>
#import "ViewController.h"
#import "RBPublisher.h"
#import "RBSubscriber.h"
#import "RBManager.h"
#import "NavSatFixMessage.h"
#import "RangeMessage.h"
#import "IMUMessage.h"

@interface ViewController () <RBManagerDelegate>
@property (weak, nonatomic) IBOutlet UILabel *latLabel;
@property (weak, nonatomic) IBOutlet UILabel *lonLabel;
@property (weak, nonatomic) IBOutlet UILabel *altLabel;
@property (weak, nonatomic) IBOutlet UILabel *altSonarLabel;

@property (weak, nonatomic) IBOutlet UISlider *xThrottleSlider;
@property (weak, nonatomic) IBOutlet UILabel *xThrottleLabel;
@property (weak, nonatomic) IBOutlet UISlider *yThrottleSlider;
@property (weak, nonatomic) IBOutlet UILabel *yThrottleLabel;
@property (weak, nonatomic) IBOutlet UIView *controlsView;

@property (weak, nonatomic) IBOutlet UISlider *zHeadingSlider;
@property (weak, nonatomic) IBOutlet UILabel *zHeadingLabel;

@property (weak, nonatomic) IBOutlet UISlider *hoverHeightSlider;
@property (weak, nonatomic) IBOutlet UILabel *hoverHeightLabel;
@property (weak, nonatomic) IBOutlet UIView *hoverHeightView;

@property (weak, nonatomic) IBOutlet UILabel *imuHeadingLabel;

@property (weak, nonatomic) IBOutlet UIButton *controlButton;
@property (nonatomic, weak) IBOutlet UIBarButtonItem *mapButton;

@property (weak, nonatomic) IBOutlet UIButton *connectivityButton;

@end

@implementation ViewController

// ros topics publishers and subcribers
RBPublisher *twistPublisher;
RBPublisher *pointPublisher;
RBSubscriber *GPSSubscriber;
RBSubscriber *sonarSubscriber;
RBSubscriber *imuSubscriber;

CLLocationCoordinate2D uavLocation;

// resend the control commands with the NSTimer' rate
NSTimer *myTimer;

bool cntrl;
// control variables
float xThrottle, yThrottle, zHeading, hoverHeight;

- (void)viewDidLoad {
    [super viewDidLoad];
    
    // connect RBManager to rosbridge
    [[RBManager defaultManager] connect:@"ws://192.168.100.3:9090"];
    
    // setup the subscribers and publishers to the specific topics
    twistPublisher = [[RBManager defaultManager] addPublisher:@"/cmd_vel" messageType:@"geometry_msgs/Twist"];
    twistPublisher.label = @"UAV Twist controller";
    
    pointPublisher = [[RBManager defaultManager] addPublisher:@"/hover_height" messageType:@"geometry_msgs/Point"];
    twistPublisher.label = @"UAV Point controller";
    
    GPSSubscriber = [[RBManager defaultManager] addSubscriber:@"/fix" responseTarget:self selector:@selector(uavGPSUpdate:) messageClass:[NavSatFixMessage class]];
    GPSSubscriber.throttleRate = 100;
    
    sonarSubscriber = [[RBManager defaultManager] addSubscriber:@"/sonar_height" responseTarget:self selector:@selector(uavSonarUpdate:) messageClass:[RangeMessage class]];
    sonarSubscriber.throttleRate = 100;
    
    imuSubscriber = [[RBManager defaultManager] addSubscriber:@"/raw_imu" responseTarget:self selector:@selector(imuUpdate:) messageClass:[IMUMessage class]];
    imuSubscriber.throttleRate = 100;
    
    // various initialisations
    cntrl = NO;
    hoverHeight = 0.0;
    
    _xThrottleSlider.continuous = YES;
    _yThrottleSlider.continuous = YES;
    _zHeadingSlider.continuous = YES;
    
    [self initLabels];
    [self zeroControl];
}

#pragma mark - subscriber's callback methods

-(void)uavGPSUpdate:(NavSatFixMessage*) message {
    // GPS callBack, fill the related labels with appropriate info
    // set the UAV location for the map display
    
    _latLabel.text = [NSString stringWithFormat:@"%.5f", [message.latitude floatValue]];
    _lonLabel.text = [NSString stringWithFormat:@"%.5f", [message.longitude floatValue]];
    _altLabel.text = [NSString stringWithFormat:@"%.5f", [message.altitude floatValue]];
    uavLocation.latitude = [message.latitude floatValue];
    uavLocation.longitude = [message.longitude floatValue];
}

-(void)imuUpdate:(IMUMessage*) message {
    // IMU callBack, fill the yaw label
    // includes quaternion to RPY conversion
    
    float x, y, z, w;
    
    x = [message.orientation.x floatValue];
    y = [message.orientation.y floatValue];
    z = [message.orientation.z floatValue];
    w = [message.orientation.w floatValue];
    
    float yaw = asin(2*(x*y + w*z));
    float roll = atan2(2*(y*w - x*z), 1 - 2*y*y - 2*z*z);
    float pitch = atan2(2*(x*w + y*z), 1 - 2*x*x - 2*z*z);

    //NSLog(@"=============x=%.5f, y=%.5f, z=%.5f, w=%.5f\n", x, y, z, w);
    //NSLog(@"=============yaw=%.5f, pitch=%.5f, roll=%.5f\n", yaw*(180/M_PI), pitch*(180/M_PI), roll*(180/M_PI));
    _imuHeadingLabel.text = [NSString stringWithFormat:@"%.5f", yaw*(180/M_PI)];
    
}

#pragma mark - RBManager Delegate methods

-(void)uavSonarUpdate:(RangeMessage*) message {
    // Sonar height callBack
    // fill the related label
    
    _altSonarLabel.text = [NSString stringWithFormat:@"%.5f", [message.range floatValue]];
}

- (void)managerDidConnect:(RBManager *)manager {
    // RBManager delegate method
    NSLog(@"Connected to ROS");
}

-(void)manager:(RBManager*)manager didFailWithError:(NSError*)error {
    // RBManager delegate method
    NSLog(@"Did fail with error");
}

-(void)manager:(RBManager*)manager didCloseWithCode:(NSInteger)code reason:(NSString *)reason wasClean:(BOOL)wasClean {
    // RBManager delegate method
    
    NSLog(@"Did close with error");
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Buttons  Actions

- (IBAction)buttonClicked:(UIButton *)sender {
    // toggle between enable and disable of the controls
    // in disable zero x,y and heading
    // keep the altitute lock
    
    if (cntrl) {
        // when no control is published
        // disable NSTimer
        [myTimer invalidate];
        [self zeroControl];
        
        cntrl = NO;
        [sender setTitle:@"Enable" forState:UIControlStateNormal];
        
    } else {
        // color the related views
        _controlsView.backgroundColor = [UIColor cyanColor];
        _hoverHeightView.backgroundColor = [UIColor purpleColor];
        
        // using the NSTimer, repeat every 0.1 seconds the transmit of the control commands
        myTimer = [NSTimer scheduledTimerWithTimeInterval: 0.1  target: self
                                                 selector: @selector(autoRepeat_Twist:) userInfo: nil repeats: YES];
         // enable the control UI
        _xThrottleSlider.enabled = YES;
        _yThrottleSlider.enabled = YES;
        _zHeadingSlider.enabled = YES;
        _hoverHeightSlider.enabled = YES;
        
        _xThrottleLabel.enabled = YES;
        _yThrottleLabel.enabled = YES;
        _zHeadingLabel.enabled = YES;
        _hoverHeightLabel.enabled = YES;
        
        cntrl = YES;
        [sender setTitle:@"Disable" forState:UIControlStateNormal];
    }
}

- (IBAction)disconnectAndClose:(id)sender {
    // disconnect the RBManager from the rosbridge and terminate
    
    [[RBManager defaultManager] disconnect];
    [NSThread sleepForTimeInterval: 1.0];
    exit(0);
}

#pragma mark - UISlider handling

- (IBAction)xSliderValueChanged:(id)sender {
    // read the linear velocity in x direction and set the apprpriate value
    
    _xThrottleLabel.text = [NSString stringWithFormat:@"x:%.5f", _xThrottleSlider.value];
    
    xThrottle = _xThrottleSlider.value;
}

- (IBAction)ySliderValueChanged:(id)sender {
    // read the linear velocity in y direction and set the apprpriate value
    
    _yThrottleLabel.text = [NSString stringWithFormat:@"y:%.5f", _yThrottleSlider.value];
    
    yThrottle = _yThrottleSlider.value;
}

- (IBAction)zSliderValueChanged:(id)sender {
    // read the angular velocity in z direction and set the apprpriate value
    // distinguish between clockwise and anticlockwise
    
    if (_zHeadingSlider.value > 0) {
        _zHeadingLabel.text = [NSString stringWithFormat:@"anticlockwise"];
    } else if (_zHeadingSlider.value < 0) {
        _zHeadingLabel.text = [NSString stringWithFormat:@"clockwise"];
    } else if (_zHeadingSlider.value == 0) {
        _zHeadingLabel.text = [NSString stringWithFormat:@""];
    }
    
    zHeading = _zHeadingSlider.value;
}

- (IBAction)hoverHeightSliderValueChanged:(id)sender {
     // read the the desired altitude hold and set the apprpriate value
    
    _hoverHeightLabel.text = [NSString stringWithFormat:@"h:%.2f", _hoverHeightSlider.value];
    
    hoverHeight = _hoverHeightSlider.value;
}


-(void)autoRepeat_Twist:(NSTimer*) t
{
    // NSTimer callback repeated every 0.1 sec when the controll is activated
    // and publish to related topics in rosbridge
    
    CGFloat xlinearVelocity = xThrottle;
    CGFloat ylinearVelocity = yThrottle;
    CGFloat zAngularVelocity = zHeading*(M_PI/180.0);
    CGFloat zHoverHeight = hoverHeight;
    
    // declare and fill the twist message for cmd_vel topic
    TwistMessage * twist = [[TwistMessage alloc] init];
    twist.linear.x = [NSNumber numberWithFloat:xlinearVelocity];
    twist.linear.y = [NSNumber numberWithFloat:ylinearVelocity];
    twist.angular.z = [NSNumber numberWithFloat:zAngularVelocity];
    
    [twistPublisher publish:twist];
    
    // declare and fill the point message for hover_height topic
    PointMessage * point = [[PointMessage alloc] init];
    point.z = [NSNumber numberWithFloat:zHoverHeight];
    
    [pointPublisher publish:point];
}

#pragma mark - UIViewControllers navigation

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // manage the UIViewControllers navigation between the central hub,
    // the map and the cam screens
    
    if ([segue.identifier isEqualToString:@"showCam"]) {
        UINavigationController *navigationController = segue.destinationViewController;
        CamViewController *camViewController = [navigationController viewControllers][0];
        camViewController.delegate = self;
    } else if ([segue.identifier isEqualToString:@"showMap"]) {
        UINavigationController *navigationController = segue.destinationViewController;
        MapViewController *mapViewController = [navigationController viewControllers][0];
        mapViewController.delegate = self;
        mapViewController.uavLocation = uavLocation;
    }
    
}

- (void)camViewControllerDidCancel: (CamViewController *)controller {
    
    [self dismissViewControllerAnimated:YES completion:nil];
}

- (void)mapViewControllerDidCancel: (MapViewController *)controller {
    
    [self dismissViewControllerAnimated:YES completion:nil];
}

#pragma mark - Helper methods

- (void)initLabels {
    // set all data labels to blank at init
    
    _hoverHeightLabel.text = [NSString stringWithFormat:@""];
    _zHeadingLabel.text = [NSString stringWithFormat:@""];
    _xThrottleLabel.text = [NSString stringWithFormat:@""];
    _yThrottleLabel.text = [NSString stringWithFormat:@""];
    _altSonarLabel.text = [NSString stringWithFormat:@""];
    _imuHeadingLabel.text = [NSString stringWithFormat:@""];
}

- (void)zeroControl {
    // zero and disable the control related entities
    
    xThrottle = 0.0;
    yThrottle = 0.0;
    zHeading = 0.0;
    
    _xThrottleSlider.enabled = NO;
    _yThrottleSlider.enabled = NO;
    _zHeadingSlider.enabled = NO;
    _hoverHeightSlider.enabled = NO;
    
    _xThrottleLabel.enabled = NO;
    _yThrottleLabel.enabled = NO;
    _zHeadingLabel.enabled = NO;
    _hoverHeightLabel.enabled = NO;
    
    _xThrottleSlider.value = 0.0;
    _yThrottleSlider.value = 0.0;
    _zHeadingSlider.value = 0.0;
    
    _xThrottleLabel.text = [NSString stringWithFormat:@"x:%.5f",0.0];
    _yThrottleLabel.text = [NSString stringWithFormat:@"y:%.5f",0.0];
    _zHeadingLabel.text = [NSString stringWithFormat:@"heading"];
    _controlsView.backgroundColor = [UIColor grayColor];
    _hoverHeightView.backgroundColor = [UIColor grayColor];
}



@end
