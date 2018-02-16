//
//  FluidDynamicsSolver_v2.swift
//  FluidDynamics
//
//  Created by Simon Gladman on 30/08/2014.
//  Copyright (c) 2014 Simon Gladman. All rights reserved.
//
//  CFD Solver based on this AS3 implementation from Oaxoa
//  http://blog.oaxoa.com/2008/01/21/actionscript-3-fluids-simulation/
//
//  Which is based on Jos Stam's paper, "Real Time Fluid Dynamics for Games"
//  http://www.dgp.toronto.edu/people/stam/reality/Research/pdf/GDC03.pdf
//
//  Used by me many times
//  http://flexmonkey.blogspot.co.uk/search/label/Computational%20Fluid%20Dynamics
//
//  Thanks to Joseph Lord for hints on optimsation
//  http://blog.human-friendly.com/

import Foundation

struct FluidDynamicsSolver_v2
{
    
    static var frameNumber : Int = 0;
    
    static let GRID_WIDTH = 200;
    static let GRID_HEIGHT = 200;
    static let DBL_GRID_HEIGHT = Double(GRID_HEIGHT);
    static let CELL_COUNT = (GRID_WIDTH + 2) * (GRID_HEIGHT + 2);
    
    static let dt = 0.075;
    static let visc = 0.0;
    static let diff = 0.0;
    static let linearSolverIterations = 2;
    
    static var d = [Double](repeating: 0, count: CELL_COUNT);
    static var dOld = [Double](repeating: 0, count: CELL_COUNT);
    static var u = [Double](repeating: 0, count: CELL_COUNT);
    static var uOld = [Double](repeating: 0, count: CELL_COUNT);
    static var v = [Double](repeating: 0, count: CELL_COUNT);
    static var vOld = [Double](repeating: 0, count: CELL_COUNT);
    static var curl = [Double](repeating: 0, count: CELL_COUNT);
    
    static func fluidDynamicsStep() -> [Double]
    {
        let startTime : CFAbsoluteTime = CFAbsoluteTimeGetCurrent();
        
        if frameNumber < 100
        {
            for i in 90 ..< 110
            {
                for j in 90 ..< 110
                {
                    let random = Int(arc4random() % 100);
                    
                    if random > frameNumber
                    {
                        d[FluidDynamicsSolver_v2.getIndex(i, j: j)] = d[FluidDynamicsSolver_v2.getIndex(i, j: j)] + Double(arc4random() % 25) / 25;
                        
                        d[FluidDynamicsSolver_v2.getIndex(i, j: j)] = d[FluidDynamicsSolver_v2.getIndex(i, j: j)] > 1 ? 1 : d[FluidDynamicsSolver_v2.getIndex(i, j: j)];
                        
                        let randomU = (Double((arc4random() % 100)) / 100) * ((arc4random() % 100) >= 50 ? -4.0 : 4.0);
                        u[FluidDynamicsSolver_v2.getIndex(i, j: j)] = randomU
                        
                        let randomV = (Double((arc4random() % 100)) / 100) * ((arc4random() % 100) >= 50 ? -4.0 : 4.5);
                        v[FluidDynamicsSolver_v2.getIndex(i, j: j)] = randomV
                        
                        let randomCurl = (Double((arc4random() % 100)) / 100) * ((arc4random() % 100) >= 50 ? -4.0 : 4.0);
                        curl[FluidDynamicsSolver_v2.getIndex(i, j: j)] = randomCurl
                    }
                }
            }
        }
        frameNumber = frameNumber + 1
        
        velocitySolver();
        densitySolver();
        
        print("CFD SOLVE:" + (NSString(format: "%.4f", CFAbsoluteTimeGetCurrent() - startTime) as String));
        
        return d;
    }
    
    static func densitySolver()
    {
        d = addSource(d, x0: dOld);
        
        swapD();
        d = diffuse(0, c: d, c0: dOld, diff: diff);
        swapD();
        
        d = advect(0, d0: dOld, du: u, dv: v);
        
        dOld = [Double](repeating: 0, count: CELL_COUNT);
    }
    
    static func velocitySolver()
    {
        //u = addSource(u, uOld);
        //v = addSource(v, vOld);
        
        addSourceUV();
        
        vorticityConfinement();
        
        // u = addSource(u, uOld);
        // v = addSource(v, vOld);
        
        addSourceUV();
        
        buoyancy();
        
        v = addSource(v, x0: vOld);
        
        swapU();
        swapV();
        
        diffuseUV();
        
        project();
        
        swapU();
        swapV();
        
        advectUV();
        
        project()
        
        uOld = [Double](repeating: 0, count: CELL_COUNT);
        vOld = [Double](repeating: 0, count: CELL_COUNT);
    }
    
    static func advectUV()
    {
        //let dt0 = dt * DBL_GRID_HEIGHT;
        
        let dt0x = dt * DBL_GRID_HEIGHT;
        let dt0y = dt * DBL_GRID_HEIGHT;
        
        for i in stride(from: GRID_WIDTH, through: 1, by: -1) //var i = GRID_HEIGHT; i >= 1; i -= 1
        {
            for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
            {
                let index = FluidDynamicsSolver_v2.getIndex(i, j :j);
                
                var x = Double(i) - dt0x * uOld[index];
                var y = Double(j) - dt0y * vOld[index];
                
                if (x > DBL_GRID_HEIGHT + 0.5)
                {
                    x = DBL_GRID_HEIGHT + 0.5;
                }
                if (x < 0.5)
                {
                    x = 0.5;
                }
                
                if (y > DBL_GRID_HEIGHT + 0.5)
                {
                    y = DBL_GRID_HEIGHT + 0.5;
                }
                
                if (y < 0.5)
                {
                    y = 0.5;
                }
                
                let i0 = Int(x);
                let i1 = i0 + 1;
                
                let j0 = Int(y);
                let j1 = j0 + 1;
                
                let s1 = x - Double(i0);
                let s0 = 1 - s1;
                let t1 = y - Double(j0);
                let t0 = 1 - t1;
                
                let i0j0 = i0 + GRID_WIDTH * j0;
                let i0j1 = i0 + GRID_WIDTH * j1;
                let i1j0 = i1 + GRID_WIDTH * j0;
                let i1j1 = i1 + GRID_WIDTH * j1;
                
                u[index] = s0 * (t0 * uOld[i0j0] + t1 * uOld[i0j1]) + s1 * (t0 * uOld[i1j0] + t1 * uOld[i1j1]);
                v[index] = s0 * (t0 * vOld[i0j0] + t1 * vOld[i0j1]) + s1 * (t0 * vOld[i1j0] + t1 * vOld[i1j1]);
            }
        }
        
    }
    
    static func advect (_ b:Int, d0:[Double], du:[Double], dv:[Double]) -> [Double]
    {
        var returnArray = [Double](repeating: 0.0, count: CELL_COUNT)
        
        //let dt0 = dt * DBL_GRID_HEIGHT;
        
        let dt0x = dt * DBL_GRID_HEIGHT;
        let dt0y = dt * DBL_GRID_HEIGHT;
        
        for i in stride(from: GRID_WIDTH, through: 1, by: -1) //var i = GRID_HEIGHT; i >= 1; i -= 1
        {
            for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
            {
                let index = FluidDynamicsSolver_v2.getIndex(i, j: j);
                
                var x = Double(i) - dt0x * du[index];
                var y = Double(j) - dt0y * dv[index];
                
                if (x > DBL_GRID_HEIGHT + 0.5)
                {
                    x = DBL_GRID_HEIGHT + 0.5;
                }
                if (x < 0.5)
                {
                    x = 0.5;
                }
                
                if (y > DBL_GRID_HEIGHT + 0.5)
                {
                    y = DBL_GRID_HEIGHT + 0.5;
                }
                
                if (y < 0.5)
                {
                    y = 0.5;
                }
                
                let i0 = Int(x);
                let i1 = i0 + 1;
                
                let j0 = Int(y);
                let j1 = j0 + 1;
                
                let s1 = x - Double(i0);
                let s0 = 1 - s1;
                let t1 = y - Double(j0);
                let t0 = 1 - t1;
                
                let i0j0 = i0 + GRID_WIDTH * j0;
                let i0j1 = i0 + GRID_WIDTH * j1;
                let i1j0 = i1 + GRID_WIDTH * j0;
                let i1j1 = i1 + GRID_WIDTH * j1;
                
                let cellValue = 0.99 * s0 * (t0 * d0[i0j0] + t1 * d0[i0j1]) + s1 * (t0 * d0[i1j0] + t1 * d0[i1j1]);
                
                returnArray[index] = cellValue;
            }
        }
        
        returnArray = setBoundry(b, x: returnArray);
        
        return returnArray;
    }
    
    // project is always on u and v....
    static func project()
    {
        var p = [Double](repeating: 0, count: CELL_COUNT);
        var div = [Double](repeating: 0, count: CELL_COUNT);
        
        for i in 1 ..< GRID_WIDTH //var i = GRID_HEIGHT; i >= 1; i -= 1
        {
            for j in 1 ..< GRID_HEIGHT //var j = GRID_HEIGHT; j >= 1; j -= 1
            {
                let index = FluidDynamicsSolver_v2.getIndex(i, j : j);
                let left = index - 1;
                let right = index + 1;
                let top = index - GRID_WIDTH;
                let bottom = index + GRID_WIDTH;
                
                div[index] = (u[right] - u[left] + v[bottom] - v[top]) * -0.5 / DBL_GRID_HEIGHT;
                
                p[index] = Double(0.0);
            }
            
        }
        
        div = setBoundry(0, x: div);
        p = setBoundry(0, x: p);
        
        p = linearSolver(0, x: p, x0: div, a: 1, c: 4);
        
        for i in stride(from: GRID_WIDTH, through: 1, by: -1) //var i = GRID_WIDTH; i >= 1; i -= 1
        {
            for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
            {
                let index = FluidDynamicsSolver_v2.getIndex(i, j : j);
                let left = index - 1;
                let right = index + 1;
                let top = index - GRID_WIDTH;
                let bottom = index + GRID_WIDTH;
                
                u[index] -= 0.5 * DBL_GRID_HEIGHT * (p[right] - p[left]);
                v[index] -= 0.5 * DBL_GRID_HEIGHT * (p[bottom] - p[top]);
            }
        }
        
        u = setBoundry(1, x: u);
        v = setBoundry(2, x: v);
    }
    
    static  func diffuseUV()
    {
        let a:Double = dt * diff * Double(CELL_COUNT);
        let c:Double = 1 + 4 * a
        
        for _ in 0 ..< linearSolverIterations
        {
            for i in stride(from: GRID_WIDTH, through: 1, by: -1) //ar i = GRID_WIDTH; i >= 1; i -= 1
            {
                for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
                {
                    let index = FluidDynamicsSolver_v2.getIndex(i, j: j);
                    let left = index - 1;
                    let right = index + 1;
                    let top = index - GRID_WIDTH;
                    let bottom = index + GRID_WIDTH;
                    
                    u[index] = (a * ( u[left] + u[right] + u[top] + u[bottom]) + uOld[index]) / c;
                    
                    v[index] = (a * ( v[left] + v[right] + v[top] + v[bottom]) + vOld[index]) / c;
                }
            }
        }
    }
    
    static func linearSolver(_ b:Int, x:[Double], x0:[Double], a:Double, c:Double) -> [Double]
    {
        var returnArray = [Double](repeating: 0.0, count: CELL_COUNT)
        
        for _ in 0 ..< linearSolverIterations
        {
            for i in stride(from: GRID_WIDTH, through: 1, by: -1) //var i = GRID_WIDTH; i >= 1; i -= 1
            {
                for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
                {
                    let index = FluidDynamicsSolver_v2.getIndex(i, j: j);
                    let left = index - 1;
                    let right = index + 1;
                    let top = index - GRID_WIDTH;
                    let bottom = index + GRID_WIDTH;
                    
                    returnArray[index] = (a * ( x[left] + x[right] + x[top] + x[bottom]) + x0[index]) / c;
                }
            }
            returnArray = setBoundry(b, x: returnArray);
        }
        
        return returnArray;
    }
    
    static func diffuse(_ b:Int, c:[Double], c0:[Double], diff:Double) -> [Double]
    {
        let a:Double = dt * diff * Double(CELL_COUNT);
        
        let returnArray = linearSolver(b, x: c, x0: c0, a: a, c: 1 + 4 * a);
        
        return returnArray
    }
    
    // buoyancy always on vOld...
    static func buoyancy()
    {
        var Tamb:Double = 0;
        let a:Double = 0.000625 //0.000625;
        let b:Double = 0.025 //0.025;
        
        
        // sum all temperatures
        for i in 1...GRID_WIDTH //var i = 1; i <= GRID_WIDTH; i += 1
        {
            for j in 1...GRID_HEIGHT //var j = 1; j <= GRID_HEIGHT; j += 1
            {
                Tamb += d[FluidDynamicsSolver_v2.getIndex(i, j: j)];
            }
        }
        
        // get average temperature
        Tamb /= Double(CELL_COUNT);
        
        // for each cell compute buoyancy force
        for i in stride(from: GRID_WIDTH, through: 1, by: -1) //var i = GRID_WIDTH; i >= 1; i -= 1
        {
            for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
            {
                let index = FluidDynamicsSolver_v2.getIndex(i, j: j);
                
                vOld[index] = a * d[index] + -b * (d[index] - Tamb);
            }
        }
    }
    
    // always on vorticityConfinement(uOld, vOld);
    static func vorticityConfinement()
    {
        for i in stride(from: GRID_WIDTH, through: 1, by: -1) //var i = GRID_WIDTH; i >= 1; i -= 1
        {
            for j in stride(from: GRID_HEIGHT, through: 1, by: -1) //var j = GRID_HEIGHT; j >= 1; j -= 1
            {
                let tt=curlf(i, j: j)
                curl[FluidDynamicsSolver_v2.getIndex(i, j: j)] = tt<0 ? tt * -1:tt;
            }
        }
        
        for i in 2 ..< GRID_WIDTH
        {
            for j in 2 ..< GRID_HEIGHT
            {
                let index = FluidDynamicsSolver_v2.getIndex(i, j: j);
                let left = index - 1;
                let right = index + 1;
                let top = index - GRID_WIDTH;
                let bottom = index + GRID_WIDTH;
                
                // Find derivative of the magnitude (n = del |w|)
                var dw_dx = (curl[right] - curl[left]) * 0.5;
                var dw_dy = (curl[bottom] - curl[top]) * 0.5;
                
                let length = hypot(dw_dx, dw_dy) + 0.000001;
                
                // N = ( n/|n| )
                dw_dx /= length;
                dw_dy /= length;
                
                let v = curlf(i, j: j);
                
                // N x w
                uOld[FluidDynamicsSolver_v2.getIndex(i, j: j)] = dw_dy * -v;
                vOld[FluidDynamicsSolver_v2.getIndex(i, j: j)] = dw_dx *  v;
            }
        }
    }
    
    static func curlf(_ i:Int, j:Int) -> Double
    {
        let index = FluidDynamicsSolver_v2.getIndex(i, j: j);
        let left = index - 1;
        let right = index + 1;
        let top = index - GRID_WIDTH;
        let bottom = index + GRID_WIDTH;
        
        let du_dy:Double = (u[bottom] - u[top]) * 0.5;
        let dv_dx:Double = (v[right] - v[left]) * 0.5;
        
        return du_dy - dv_dx;
    }
    
    static func setBoundry(_ b:Int, x:[Double]) -> [Double]
    {
        let returnArray = x;
        
        return returnArray;
        
        /*
        for var i = GRID_HEIGHT; i >= 1; i--
        {
        if(b==1)
        {
            returnArray[getIndex(  0, j :i  )] = -x[getIndex(1, j: i)];
            returnArray[getIndex(GRID_HEIGHT+1, j: i  )] = -x[getIndex(GRID_HEIGHT, j: i)];
        }
        else
        {
            returnArray[getIndex(  0, j: i  )] = x[getIndex(1, j :i)];
            returnArray[getIndex(GRID_HEIGHT+1, j :i  )] = x[getIndex(GRID_HEIGHT,j : i)];
        }
        
        if(b==2)
        {
            returnArray[getIndex(  i, j :0  )] = -x[getIndex(i, j : 1)];
            returnArray[getIndex(  i, j : GRID_HEIGHT+1)] = -x[getIndex(i, j : GRID_HEIGHT)];
        }
        else
        {
            returnArray[getIndex(  i, j : 0  )] = x[getIndex(i, j : 1)];
            returnArray[getIndex(  i, j : GRID_HEIGHT+1)] = x[getIndex(i, j : GRID_HEIGHT)];
        }
        }
        
        returnArray[getIndex(0, j : 0)] = 0.5 * (x[getIndex(1, j : 0  )] + x[getIndex(0, j : 1)]);
        returnArray[getIndex(0, j : GRID_HEIGHT+1)] = 0.5 * (x[getIndex(1, j : GRID_HEIGHT+1)] + x[getIndex(  0, j : GRID_HEIGHT)]);
        returnArray[getIndex(GRID_HEIGHT+1, j : 0)] = 0.5 * (x[getIndex(GRID_HEIGHT, j : 0)] + x[getIndex(GRID_HEIGHT+1, j : 1)]);
        returnArray[getIndex(GRID_HEIGHT+1, j : GRID_HEIGHT+1)] = 0.5 * (x[getIndex(GRID_HEIGHT, j : GRID_HEIGHT+1)] + x[getIndex(GRID_HEIGHT+1, j : GRID_HEIGHT)]);
        
        return returnArray;
        */
    }
    
    static func addSourceUV()
    {
        for i in stride(from: CELL_COUNT - 1, through: 0, by: -1) //var i = CELL_COUNT - 1; i >= 0; i -= 1
        {
            u[i] = u[i] + dt * uOld[i];
            v[i] = v[i] + dt * vOld[i];
        }
    }
    
    static func addSource(_ x:[Double], x0:[Double]) -> [Double]
    {
        var returnArray = [Double](repeating: 0.0, count: CELL_COUNT)
        
        for i in stride(from: CELL_COUNT - 1, through: 0, by: -1) //var i = CELL_COUNT - 1; i >= 0; i -= 1
        {
            returnArray[i] = x[i] + dt * x0[i];
        }
        
        return returnArray;
    }
    
    
    static func swapD()
    {
        let tmp = d;
        d = dOld;
        dOld = tmp;
    }
    
    static func swapU()
    {
        let tmp = u;
        u = uOld;
        uOld = tmp;
    }
    
    static func swapV()
    {
        let tmp = v;
        v = vOld;
        vOld = tmp;
    }
    
    static func getIndex(_ i : Int, j : Int) -> Int
    {
        return i + (FluidDynamicsSolver_v2.GRID_WIDTH) * j;
    }
    
}
