//
//  RangeMessage.m
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 07/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "RangeMessage.h"
#import "HeaderMessage.h"

@implementation RangeMessage
@synthesize header, radiation_type, field_of_view, min_range, max_range, range;

-(void)setDefaults {
    self.header = [[HeaderMessage alloc] init];
}

@end
