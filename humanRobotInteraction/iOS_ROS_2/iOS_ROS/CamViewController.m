//
//  CamViewController.m
//  iOS_ROS
//
//  Created by Angelos Plastropoulos on 08/08/2015.
//  Copyright (c) 2015 Angelos Plastropoulos. All rights reserved.
//

#import "CamViewController.h"

@interface CamViewController ()
@property (weak, nonatomic) IBOutlet UIWebView *myWebView;

@end

@implementation CamViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    // Set the NSTimer to repeatedly call the refresh action of the UIWebView
    NSTimer *myTimer = [NSTimer scheduledTimerWithTimeInterval: 0.5  target: self
                                                      selector: @selector(webViewReload:) userInfo: nil repeats: YES];
    
    // Connect the UIWebView with the content transimeted from mjpeg server in ros
    // This server publishes images from the front cam (topic)
    NSURLRequest* urlRequest = [NSURLRequest requestWithURL:[NSURL URLWithString:@"http://192.168.100.3:8080/stream?topic=/front_cam/camera/image"]];
    
    [_myWebView loadRequest:urlRequest];
    
}

-(void)webViewReload:(NSTimer*) t {
    // NSTimer callback method, reload the UIWebView to update the image
    
    [_myWebView reload];
}


#pragma mark - Actions

- (IBAction)cancel:(id)sender {
    // return to central hub screen
    
    [self.delegate camViewControllerDidCancel:self];
}

@end
