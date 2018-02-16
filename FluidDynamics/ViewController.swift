//
//  ViewController.swift
//  FluidDynamics
//
//  Created by Simon Gladman on 26/08/2014.
//  Copyright (c) 2014 Simon Gladman. All rights reserved.
//

import UIKit

class ViewController: UIViewController {
    
    var densities : [Double] = [Double](repeating: 0, count: FluidDynamicsSolver_v2.CELL_COUNT);
    
    @IBOutlet var uiImageView: UIImageView!
    var uiImage : UIImage?;
    
    required init?(coder aDecoder: NSCoder)
    {
        super.init(coder: aDecoder);
    }
    
    override func viewDidLoad()
    {
        super.viewDidLoad()
        
        dispatchSolve();
    }
    
    @IBAction func buttonClick(_ sender: AnyObject)
    {
        FluidDynamicsSolver_v2.frameNumber = 0;
    }
    
    var previousTouchX : Int?;
    var previousTouchY : Int?;
 
 
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        let touch:CGPoint? = event?.allTouches?.first?.location(in: uiImageView)
        
        if let touchX = touch?.x
        {
            if let touchY = touch?.y
            {
                let scaledX = touchX / 3
                let scaledY = touchY / 3
                let intScaledX = Int(scaledX)
                let intScaledY = Int(scaledY)
                
                for i in intScaledX - 3 ..< intScaledX + 3
                {
                    for j in intScaledY - 3 ..< intScaledY + 3
                    {
                        let targetIndex = FluidDynamicsSolver_v2.getIndex(Int(i), j: Int(j));
                        
                        if targetIndex > 0 && targetIndex < FluidDynamicsSolver_v2.CELL_COUNT
                        {
                            FluidDynamicsSolver_v2.d[targetIndex] = 0.9;
                            
                            if let ptx = previousTouchX
                            {
                                if let pty = previousTouchY
                                {
                                    FluidDynamicsSolver_v2.u[targetIndex] = FluidDynamicsSolver_v2.u[targetIndex] + Double((Int(scaledX) - ptx))
                                    FluidDynamicsSolver_v2.v[targetIndex] = FluidDynamicsSolver_v2.v[targetIndex] + Double((Int(scaledY) - pty))
                                }
                            }
                        }
                    }
                }
                previousTouchX = Int(scaledX);
                previousTouchY = Int(scaledY);
            }
        }
    }
    
    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?)
    {
        previousTouchX = nil;
        previousTouchY = nil;
    }
    
    func dispatchSolve()
    {
        Async.background
            {
                self.densities = FluidDynamicsSolver_v2.fluidDynamicsStep()
            }
            .main
            {
                self.dispatchRender();
                
                self.dispatchSolve();
        }
    }
    
    func dispatchRender()
    {
        Async.background
            {
                self.uiImage = renderFluidDynamics(self.densities);
            }
            .main
            {
                self.uiImageView.image = self.uiImage;
        }
    }
    

    
}

