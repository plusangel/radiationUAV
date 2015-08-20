//
//  MapViewController.h
//  test
//
//  Created by Angelos Plastropoulos on 20/06/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <MapKit/MapKit.h>
#import "BLE.h"

@class MapViewController;

@protocol MapViewControllerDelegate <NSObject>
- (void)mapViewControllerDidCancel:(MapViewController *)controller withStatus:(BOOL)status;

@end

@interface MapViewController : UIViewController <BLEDelegate>
@property (weak, nonatomic) NSMutableArray *myArrayOfMeasurements;
@property (weak, nonatomic) BLE *ble;

@property (weak, nonatomic) id <MapViewControllerDelegate> delegate;

@property (assign, nonatomic) unsigned maxRange;
@property (strong, nonatomic) NSNumber* mappingMthd;
@property (assign, nonatomic) unsigned circleRadius;

- (IBAction)cancel:(id)sender;

@end
