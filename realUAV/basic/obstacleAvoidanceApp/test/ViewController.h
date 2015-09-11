//
//  ViewController.h
//  test
//
//  Created by Angelos Plastropoulos on 20/06/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "SettingsViewController.h"
#import "BLE.h"

@interface ViewController : UIViewController <SettingsViewControllerDelegate, BLEDelegate>
@property (strong, nonatomic) BLE *ble;

@property (assign, nonatomic) unsigned fwdDistance;
@property (assign, nonatomic) unsigned sdDistance;

@end


