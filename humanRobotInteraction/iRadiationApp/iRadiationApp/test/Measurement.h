//
//  Measurement.h
//  iOS_ADK_BLE
//
//  Created by Angelos Plastropoulos on 05/07/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <MapKit/MapKit.h>

@interface Measurement : NSObject

@property (assign, nonatomic) CLLocationCoordinate2D location;
@property (assign, nonatomic) unsigned cps;

@end
