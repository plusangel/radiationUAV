//
//  ViewController.h
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 05/07/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "RBManager.h"
#import "CamViewController.h"
#import "MapViewController.h"

@interface ViewController : UIViewController <CamViewControllerDelegate, MapViewControllerDelegate>

@end

