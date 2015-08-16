//
//  RangeMessage.h
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 07/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "RBMessage.h"
@class HeaderMessage;

@interface RangeMessage : RBMessage {
    HeaderMessage * header;

    NSNumber * radiation_type;
    NSNumber * field_of_view;
    NSNumber * min_range;
    NSNumber * max_range;
    
    NSNumber * range;

}

@property (nonatomic, strong) HeaderMessage * header;
@property (nonatomic, strong) NSNumber * radiation_type;
@property (nonatomic, strong) NSNumber * field_of_view;
@property (nonatomic, strong) NSNumber * min_range;
@property (nonatomic, strong) NSNumber * max_range;
@property (nonatomic, strong) NSNumber * range;

@end
