//
//  FluidDynamicsRenderer.swift
//  FluidDynamics
//
//  Created by Simon Gladman on 30/08/2014.
//  Copyright (c) 2014 Simon Gladman. All rights reserved.
//
//
//  Based on work by Joseph Lord
//  http://blog.human-friendly.com/

import Foundation
import UIKit

private let rgbColorSpace = CGColorSpaceCreateDeviceRGB()
private let bitmapInfo:CGBitmapInfo = CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedFirst.rawValue)

private func imageFromARGB32Bitmap(_ pixels:[PixelData], width:Int, height:Int)->UIImage
{
    let bitsPerComponent:Int = 8
    let bitsPerPixel:Int = 32
    
    var data = pixels // Copy to mutable []
    //let providerRef = CGDataProvider(data: CFData(bytes: UnsafePointer<UInt8>(&data), count: data.count * sizeof(PixelData)))
    
    let providerRef = CGDataProvider(data: NSData(bytes: &data, length: data.count * MemoryLayout<PixelData>.size))
    
    let cgim = CGImage(width: width, height: height, bitsPerComponent: bitsPerComponent, bitsPerPixel: bitsPerPixel, bytesPerRow: width * Int(MemoryLayout<PixelData>.size), space: rgbColorSpace,	bitmapInfo: bitmapInfo, provider: providerRef!, decode: nil, shouldInterpolate: true, intent: CGColorRenderingIntent.defaultIntent)
    
    return UIImage(cgImage: cgim!);
}

func renderFluidDynamics(_ densities : [Double]) -> UIImage
{
    var pixelArray = [PixelData](repeating: PixelData(a: 255, r:0, g: 0, b: 0), count: densities.count);
    
    for i in 0 ..< FluidDynamicsSolver_v2.CELL_COUNT
    {
        let pixelValue = UInt8(255 * densities[i]);
        
        pixelArray[i].r = pixelValue;
        pixelArray[i].g = pixelValue;
        pixelArray[i].b = pixelValue;
    }
    
    let outputImage = imageFromARGB32Bitmap(pixelArray, width: FluidDynamicsSolver_v2.GRID_WIDTH, height: FluidDynamicsSolver_v2.GRID_HEIGHT)
    
    return outputImage;
}

struct PixelData
{
    var a:UInt8 = 255
    var r:UInt8
    var g:UInt8
    var b:UInt8
}
