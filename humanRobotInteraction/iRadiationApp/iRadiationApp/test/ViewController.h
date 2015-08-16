//
//  ViewController.h
//  test
//
//  Created by Angelos Plastropoulos on 20/06/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "MapViewController.h"
#import "BLE.h"

@interface ViewController : UIViewController <MapViewControllerDelegate, BLEDelegate>
@property (strong, nonatomic) BLE *ble;
@property (strong, nonatomic) NSMutableArray *myArrayMeasurements;

@end

