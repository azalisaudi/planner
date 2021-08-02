//
// Author: Azali Saudi
// Date Created : 30 Dec 2016
// Last Modified: 02 Aug 2021
// Task: The Solver implementation of SOR.
//

import java.util.ArrayList;
import java.awt.image.*;
import javax.imageio.*;
import java.io.*;
import java.awt.Color;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Queue;

public class Solver {
    static final int WALL_VALUE = 1;
    static final int GOAL_VALUE = 0;
    static final double FREE_VALUE = 1.19;
    static final double BOUNDARY_VALUE = 1.20;

    static final double EPSILON = 1.0e-14;

    //Matrix variables
    double[] U;
    double[] V;
    int[] W;
    
    int[] RB;    

    double[] Lo;
    double[] Up;

    int Nx,Ny;

    public Queue<Point> path;

	public Solver(BufferedImage img, int gx, int gy) {
		Nx = img.getWidth();
		Ny = img.getHeight();

		U = new double[Nx*Ny];
		V = new double[Nx*Ny];
		W = new int[Nx*Ny];

        RB= new int[Nx*Ny];

		Lo = new double[Nx*Ny];
		Up = new double[Nx*Ny];

		path = new LinkedList<Point>();

		//
		// Initialize the matrix U, V, W.
		//
		int rgb;
		for(int y = 0; y < Ny; y++)
		for(int x = 0; x < Nx; x++) {
			rgb = img.getRGB(x, y);
			rgb = rgb & 0x00FFFFFF;
			// The boundary wall
			if(rgb == 0) {
				U[x+y*Nx] = V[x+y*Nx] = BOUNDARY_VALUE;
				W[x+y*Nx] = WALL_VALUE;

				Lo[x+y*Nx] = Up[x+y*Nx] = BOUNDARY_VALUE;
			}

			// The goal point
			else if((x == gx) && (y == gy)) {
				U[x+y*Nx] = V[x+y*Nx] = GOAL_VALUE;
				W[x+y*Nx] = WALL_VALUE;

				Lo[x+y*Nx] = Up[x+y*Nx] = GOAL_VALUE;
			}

			// The free space
			else { //if(rgb == 0xFFFFFF) {
				U[x+y*Nx] = V[x+y*Nx] = FREE_VALUE;
				Lo[x+y*Nx] = Up[x+y*Nx] = FREE_VALUE;
			}
		}

		// Make the 8 nyboring goal points
		int dP[][] = {{-1,0}, {1,0}, {0,-1}, {0,1}, {-1,-1}, {1,-1}, {-1,1}, {1,1}};
		for(int k = 0; k < 8; k++) {
			int x = gx + dP[k][0];
			int y = gy + dP[k][1];
			U[x+y*Nx] = V[x+y*Nx] = GOAL_VALUE;
			W[x+y*Nx] = WALL_VALUE;
			
			Lo[x+y*Nx] = Up[x+y*Nx] = GOAL_VALUE;
		}
	}

	public void updateMatrix() {
		for(int y = 1; y < Ny-1; y++)
		for(int x = 1; x < Nx-1; x++)
			if(W[x+y*Nx] != WALL_VALUE)
			{
				U[x+y*Nx] = V[x+y*Nx];
			}
	}
	
	public void doSOR(double w) {
		for(int y = 1; y < Ny-1; y++)
		for(int x = 1; x < Nx-1; x++)
			if(W[x+y*Nx] != WALL_VALUE)
			{
				V[x+y*Nx] = w*0.25 * (V[x-1+y*Nx] + U[x+1+y*Nx] + V[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
			}
	}

	public boolean checkConverge() {
		double err = 0.0;
		int k = 0;
		for(int y = 1; y < Ny-1; y++)
		for(int x = 1; x < Nx-1; x++)
			if(U[x+y*Nx] != V[x+y*Nx])
			if(W[x+y*Nx] != WALL_VALUE) {
				err += Math.abs(1-U[x+y*Nx]/V[x+y*Nx]);
				++k;
			}
		if(k > 0) err /= k;
		return (err < EPSILON);
	}

	public void runGDS(BufferedImage img, int x, int y) {
		int minx = x, miny = y;

		path.clear();
		while(true) {
			if(V[(x-1)+y*Nx] < V[minx+miny*Nx]) { minx = x-1; miny = y; }
			if(V[x+(y-1)*Nx] < V[minx+miny*Nx]) { minx = x; miny = y-1; }
			if(V[(x+1)+y*Nx] < V[minx+miny*Nx]) { minx = x+1; miny = y; }
			if(V[x+(y+1)*Nx] < V[minx+miny*Nx]) { minx = x; miny = y+1; }

			if(V[x-1+(y-1)*Nx] < V[minx+miny*Nx]) { minx = x-1; miny = y-1; }
			if(V[x+1+(y-1)*Nx] < V[minx+miny*Nx]) { minx = x+1; miny = y-1; }
			if(V[x-1+(y+1)*Nx] < V[minx+miny*Nx]) { minx = x-1; miny = y+1; }
			if(V[x+1+(y+1)*Nx] < V[minx+miny*Nx]) { minx = x+1; miny = y+1; }

			path.add(new Point(minx, miny));

			// Opps, we stuck
			if((minx == x) && (miny == y)) break;

			// The goal is found
			if(V[minx+miny*Nx] == GOAL_VALUE) break;

			x = minx; y = miny;
		}
	}

	public void printMatrix() {
		for(int y = 0; y < Ny; y++) {
			for(int x = 0; x < Nx; x++)
				System.out.print(String.format("%e ", U[x+y*Nx]));
			System.out.println();
		}
	}
}
