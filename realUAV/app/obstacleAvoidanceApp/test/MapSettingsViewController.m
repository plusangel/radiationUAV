//
//  MapSettingsViewController.m
//  iOS_ADK_BLE
//
//  Created by Angelos Plastropoulos on 12/07/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "MapSettingsViewController.h"

@interface MapSettingsViewController ()<UITextFieldDelegate>
@property (weak, nonatomic) IBOutlet UITextField *txtCPSmax;
@property (weak, nonatomic) IBOutlet UILabel *lblColor1;
@property (weak, nonatomic) IBOutlet UILabel *lblColor2;
@property (weak, nonatomic) IBOutlet UILabel *lblColor3;
@property (weak, nonatomic) IBOutlet UILabel *lblColor4;
@property (weak, nonatomic) IBOutlet UILabel *lblColor5;
@property (weak, nonatomic) IBOutlet UILabel *lblColor6;
@property (weak, nonatomic) IBOutlet UILabel *lblColor7;

@property (weak, nonatomic) IBOutlet UISegmentedControl *mappingMethodSegControl;

@end

@implementation MapSettingsViewController

#pragma mark - Support internal functionality

- (void)viewDidLoad {
    [super viewDidLoad];
    
    //NSLog(@"Max cps %d", _maxRange);
    _mappingMethodSegControl.selectedSegmentIndex = _mappingMthd;
    
    _txtCPSmax.text = [NSString stringWithFormat:@"%d", _maxRange];
    [self setColourLabels];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Actions
- (IBAction)save:(id)sender {
    [self.delegate mapSettingsViewControllerDidSave:self withCPSmax:[_txtCPSmax.text intValue] withMapMthd:_mappingMthd];
}

- (IBAction)cancel:(id)sender {
    [self.delegate mapSettingsViewControllerDidCancel:self];
}

- (IBAction)indexChanged:(UISegmentedControl *)sender {
    // allow the user to select the mapping method
    
    switch (_mappingMethodSegControl.selectedSegmentIndex)
    {
        case 0:
            _mappingMthd = _mappingMethodSegControl.selectedSegmentIndex;
            NSLog(@"by foot");
            break;
        case 1:
            _mappingMthd = _mappingMethodSegControl.selectedSegmentIndex;
            NSLog(@"by car");
            break;
        case 2:
            _mappingMthd = _mappingMethodSegControl.selectedSegmentIndex;
            NSLog(@"by uav");
            break;
        default:
            break; 
    }
}


-(BOOL) textFieldShouldReturn:(UITextField *)textField {
    // get the max range value submitted by the user
    _maxRange = [_txtCPSmax.text intValue];
    [textField resignFirstResponder];
    [self setColourLabels];
    return YES;
}

#pragma mark - helper methods

-(void) setColourLabels {
    // update the colour array on the screen to reflect the user's selection
    
    int unit = _maxRange/7;
    
    _lblColor1.text = [NSString stringWithFormat:@"%d",unit];
    _lblColor2.text = [NSString stringWithFormat:@"%d",2*unit];
    _lblColor3.text = [NSString stringWithFormat:@"%d",3*unit];
    _lblColor4.text = [NSString stringWithFormat:@"%d",4*unit];
    _lblColor5.text = [NSString stringWithFormat:@"%d",5*unit];
    _lblColor6.text = [NSString stringWithFormat:@"%d",6*unit];
    _lblColor7.text = [NSString stringWithFormat:@"%d",_maxRange];
}

@end
