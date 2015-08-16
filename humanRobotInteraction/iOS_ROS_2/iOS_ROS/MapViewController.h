//
//  MapViewController.h
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 08/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <MapKit/MapKit.h>
@class MapViewController;

@protocol MapViewControllerDelegate <NSObject>
- (void)mapViewControllerDidCancel:(MapViewController *)controller;

@end

@interface MapViewController : UIViewController
@property (weak, nonatomic) id <MapViewControllerDelegate> delegate;
@property (assign, nonatomic) CLLocationCoordinate2D uavLocation;

@end
