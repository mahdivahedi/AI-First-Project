import java.awt.*;
import java.io.FileNotFoundException;


public class Main {

    public static void main(String[] args) throws FileNotFoundException {
        PathFinder p = new PathFinder("test1.txt");
        var path = p.aStar(p.findNearestButter());
        System.out.println("********************************");
        path.forEach(System.out::println);
    }
}
