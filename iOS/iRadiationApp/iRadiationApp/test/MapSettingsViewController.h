//
//  MapSettingsViewController.h
//  iOS_ADK_BLE
//
//  Created by Angelos Plastropoulos on 12/07/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import <UIKit/UIKit.h>

@class MapSettingsViewController;

@protocol MapSettingsViewControllerDelegate <NSObject>
- (void)mapSettingsViewControllerDidCancel:(MapSettingsViewController *)controller;
- (void)mapSettingsViewControllerDidSave:(MapSettingsViewController *)controller withCPSmax:(unsigned)cpsValue withMapMthd:(unsigned)mthdValue;

@end

@interface MapSettingsViewController : UIViewController

@property (weak, nonatomic) id <MapSettingsViewControllerDelegate> delegate;
@property (assign, nonatomic) unsigned maxRange;
@property (assign, nonatomic) unsigned mappingMthd;


- (IBAction)indexChanged:(UISegmentedControl *)sender;

@end
