//
//  SettingsViewController.h
//  iOS_BLE_Lidar
//
//  Created by Angelos Plastropoulos on 14/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>

@class SettingsViewController;

@protocol SettingsViewControllerDelegate <NSObject>
- (void)settingsViewControllerDidCancel:(SettingsViewController *)controller;
- (void)settingsViewControllerDidSave:(SettingsViewController *)controller withFWdist:(unsigned)fwdValue withSDdist:(unsigned)sdValue;

@end

@interface SettingsViewController : UIViewController

@property (weak, nonatomic) id <SettingsViewControllerDelegate> delegate;
@property (assign, nonatomic) unsigned fwdDist;
@property (assign, nonatomic) unsigned sdDist;

@end
