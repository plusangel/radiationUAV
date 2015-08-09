//
//  MapViewController.m
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 08/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "MapViewController.h"
#define METERS_PER_MILE 1609.344

@interface MapViewController () <MKMapViewDelegate>
@property (weak, nonatomic) IBOutlet MKMapView *mapView;

@end

@implementation MapViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    _mapView.delegate = self;
    
    // define the zoom and the region of the map view
    MKCoordinateRegion viewRegion = MKCoordinateRegionMakeWithDistance(_uavLocation, 0.5*METERS_PER_MILE, 0.5*METERS_PER_MILE);
    [_mapView setRegion:viewRegion animated:YES];
    
    [self drawUAV:_uavLocation];

}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Actions

- (IBAction)cancel:(id)sender {
    // return to central hub screen
    
    [self.delegate mapViewControllerDidCancel:self];
}

- (void)drawUAV:(CLLocationCoordinate2D) loc {
    // create a circle overlay to display the exact location of the UAV
    
    CLLocationCoordinate2D center = loc;
    MKCircle *circle = [MKCircle circleWithCenterCoordinate:center radius:10];
    
    [self.mapView addOverlay:circle];
}

- (MKOverlayView *)mapView:(MKMapView *)mapView viewForOverlay:(id<MKOverlay>)overlay {
    // handle the overlay
    // draw a cirle in the location specified
    
    MKCircleView *circleView = [[MKCircleView alloc] initWithOverlay:overlay];
    
    [circleView setFillColor:[UIColor redColor]];
    [circleView setStrokeColor:[UIColor blackColor]];
    [circleView setAlpha:1.0f];
    return circleView;
}
@end
