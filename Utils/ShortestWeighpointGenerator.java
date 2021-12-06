package org.firstinspires.ftc.teamcode.Utils;

import java.util.*;

public class ShortestWeighpointGenerator {
    static PriorityQueue<node> pq = new PriorityQueue();
    static int [][] grid = new int[256][256];

    // Frame of reference is me on the back of the field
    // N, S, E, W, NE, NW, SE, SW
    static final int[] dx = {0, 0, -1, 1, 1, 1, -1, -1};
    static final int[] dy = {-1, 1, 0, 0, 1, -1, 1,  -1};

    public static ArrayList<Point> generatePath(Point sc, Point dest){
        // dikstras
        ArrayList<Point> weighPoints = new ArrayList<>();
        pq.add(new node(sc, 0));

        while(!pq.isEmpty()){

        }

        return weighPoints;
    }



}

class node implements Comparable<node>{
    Point p;
    int cost;

    public node(Point p, int cost){
        this.p = p;
        this.cost = cost;
    }


    @Override
    public int compareTo(node o) {
        return Integer.compare(this.cost, o.cost);
    }

}
