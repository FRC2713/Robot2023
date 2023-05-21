package frc.robot.util;

import lombok.Getter;
import java.util.ArrayList;

public class CircularBuffer<T> {
    @Getter private final int size;
    private final ArrayList<T> list;

    public CircularBuffer(int size) {
        if(size < 1) {
            throw new IllegalArgumentException("Too small size");
        }
        this.size = size;
        list = new ArrayList<>();
    }

    public boolean push(T ele) {
        if(list.size() < size) {
            list.add(ele);
        }
        else {
            list.remove(0);
            list.add(ele);
        }
        System.out.println(list.toString());
        return !(list.size() < size);
    }

    public int realSize() {
        return list.size();
    }

    public Object[] toArray() {
        return list.toArray();
    }

    public ArrayList<T> toArrayList() {
        return list;
    }

    public T get(int index) {
        return list.get(index);
    }
}

