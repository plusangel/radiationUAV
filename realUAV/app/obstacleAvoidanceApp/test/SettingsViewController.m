//
//  SettingsViewController.m
//  iOS_BLE_Lidar
//
//  Created by Angelos Plastropoulos on 14/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "SettingsViewController.h"

@interface SettingsViewController () <UITextFieldDelegate>
@property (weak, nonatomic) IBOutlet UITextField *txtFWdist;
@property (weak, nonatomic) IBOutlet UITextField *txtSDdist;

@property (nonatomic, weak) IBOutlet UIBarButtonItem *bbtnCancel;
@property (nonatomic, weak) IBOutlet UIBarButtonItem *bbtnSave;

@end

@implementation SettingsViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    _txtFWdist.text = [NSString stringWithFormat:@"%d", _fwdDist];
    _txtSDdist.text = [NSString stringWithFormat:@"%d", _sdDist];
    
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Actions
- (IBAction)save:(id)sender {
    [self.delegate settingsViewControllerDidSave:self withFWdist:[_txtFWdist.text intValue] withSDdist:[_txtSDdist.text intValue]];
}

- (IBAction)cancel:(id)sender {
    [self.delegate settingsViewControllerDidCancel:self];
}

-(BOOL) textFieldShouldReturn:(UITextField *)textField {
    NSLog(@"textFieldShouldReturn");
    if (textField.tag == 0) {
        _fwdDist = [_txtFWdist.text intValue];
        [textField resignFirstResponder];
    } else if (textField.tag == 1) {
        _sdDist = [_txtSDdist.text intValue];
        [textField resignFirstResponder];
    }
    
    NSLog(@"fw:%d sd:%d", _fwdDist, _sdDist);
    return YES;
}



@end
