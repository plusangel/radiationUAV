//
//  CamViewController.h
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 08/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>
@class CamViewController;

@protocol CamViewControllerDelegate <NSObject>
- (void)camViewControllerDidCancel:(CamViewController *)controller;

@end

@interface CamViewController : UIViewController
@property (weak, nonatomic) id <CamViewControllerDelegate> delegate;

@end
