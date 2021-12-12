package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayDeque;
import java.util.ArrayList;

// resource: https://realitybytes.blog/2017/07/11/graph-based-path-planning-dijkstras-algorithm/
// resource: https://cp-algorithms.com/graph/01_bfs.html

public class ShortestWeighpointGenerator {
    static ArrayDeque<node> queue = new ArrayDeque<>();
    static int [][] grid = new int[256+1][256+1]; // derive from image
    static int [][] dist = new int[256+1][256+1];
    static node [][] prev = new node[265+1][256+1];

    // Frame of reference is me on the back of the field
    // N, S, E, W, NE, NW, SE, SW
    static final int[] dx = {0, 0, -1, 1, 1, 1, -1, -1};
    static final int[] dy = {-1, 1, 0, 0, 1, -1, 1,  -1};

    public ShortestWeighpointGenerator(){
        // init image grid
    }

    // Floyd warshalls? for dp computation.
    public ArrayList<Point> generatePath(Point sc, Point dest){
        // dikstras (for now) --> Replaced with 0/1 BFS
        // todo: replace with Floyd warshalls version of 0/1 bfs
        ArrayList<Point> weighPoints = new ArrayList<>();
        queue.addFirst(new node((int) sc.xP, (int) sc.yP));

        while(!queue.isEmpty()){
            node curr = queue.poll();
            assert curr != null;
            if(dist[curr.x][curr.y] > 0) // path to node already generated
                continue;

            for (int k = 0; k < dx.length; k++) {
                int nx = curr.x + dx[k];
                int ny = curr.y + dy[k];
                int res = updatePose(new node(nx, ny));

                if (res == -1) {
                    continue;
                } else if (res == 1){
                    prev[nx][ny] = new node(curr.x, curr.y);
                    dist[nx][ny] = dist[curr.x][curr.y] + 1;

                    // N, S, E, W, NE, NW, SE, SW
                    String travDir = getTravDir(k);
                    queue.addLast(new node(nx, ny, travDir));
                } else {
                    prev[nx][ny] = new node(curr.x, curr.y);
                    dist[nx][ny] = dist[curr.x][curr.y];

                    String travDir = getTravDir(k);
                    queue.addFirst(new node(nx, ny, travDir));
                }
            }
        }

        // reconstruct the path


        return weighPoints;
    }

    /*
     * Returns the cost of the path taken [1 = Object Collision; -1 = Out of Bounds].
     * creates a 18 x 18 padding around the point to ensure collisions do not occur
     */

    public int updatePose(node currPose){
        for (int i = currPose.x - 9; i <= currPose.x + 9; i++) {
            for (int j = currPose.y - 9; j <= currPose.y + 9; j++) {
                if(i < 0 || j < 0 || i > 256 || j > 256+1) return -1; // out of bounds
                if(grid[i][j] == 1) return 1; // Object collision
            }
        }
        return 0; // no collisions
    }

    public String getTravDir(int k){
        String travDir;
        if(k == 0){
            travDir = "N";
        }else if(k == 1){
            travDir = "S";
        }else if(k == 2){
            travDir = "E";
        }else if(k == 3){
            travDir = "W";
        }else if(k == 4){
            travDir = "NE";
        }else if(k == 5){
            travDir = "NW";
        }else if(k == 6){
            travDir = "SE";
        }else{
            travDir = "SE";
        }

        return travDir;
    }

    static class node{
        int x;
        int y;
        String dir;

        public node(int x, int y){
            this.x = x;
            this.y = y;
        }

        public node(int x, int y, String dir) {
            this.x = x;
            this.y = y;
            this.dir = dir;
        }
    }
}


