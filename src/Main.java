import java.awt.*;
import java.io.IOException;


public class Main {

    public static void main(String[] args) throws IOException {
        PathFinder p = new PathFinder("test4.txt");
        p.findRoute();
        p.printMap();
    }
}
