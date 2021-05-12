import java.awt.*;
import java.io.FileNotFoundException;
import java.io.IOException;


public class Main {

    public static void main(String[] args) throws IOException {
        PathFinder p = new PathFinder("test5.txt");
        p.findRoute();
        p.printMap();
//        var path = p.aStar(p.findNearestButter());
//        System.out.println("********************************");
//        path.forEach(System.out::println);
    }
}
