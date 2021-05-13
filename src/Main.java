import java.awt.*;
import java.io.IOException;


public class Main {

    public static void main(String[] args) throws IOException {
        PathFinder p = new PathFinder("test3.txt");
//        p.findRouteBBFS();
        p.findRouteAStar();
    }
}
