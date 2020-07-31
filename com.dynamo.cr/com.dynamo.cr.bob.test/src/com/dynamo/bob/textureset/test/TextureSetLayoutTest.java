// Copyright 2020 The Defold Foundation
// Licensed under the Defold License version 1.0 (the "License"); you may not use
// this file except in compliance with the License.
// 
// You may obtain a copy of the License, together with FAQs at
// https://www.defold.com/license
// 
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

package com.dynamo.bob.textureset.test;

import static org.hamcrest.CoreMatchers.is;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.assertFalse;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import org.junit.Test;

import com.dynamo.bob.textureset.TextureSetLayout;
import com.dynamo.bob.textureset.TextureSetLayout.Layout;
import com.dynamo.bob.textureset.TextureSetLayout.Rect;
import com.dynamo.bob.textureset.TextureSetLayout.Grid;

public class TextureSetLayoutTest {

    void assertRect(Layout layout, int i, String id, int index, int x, int y) {
        Rect r = layout.getRectangles().get(i);
        assertThat(r.id, is(id));
        assertThat(r.index, is(index));
        assertThat(r.x, is(x));
        assertThat(r.y, is(y));
    }

    private static Layout packedLayout(int margin, List<Rect> rectangles) {
        return TextureSetLayout.packedLayout(margin, rectangles, true);
    }

    private static Layout gridLayout(int margin, List<Rect> rectangles, Grid gridSize) {
        return TextureSetLayout.gridLayout(margin, rectangles, gridSize);
    }

    @Test
    public void testEmpty() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList();

        Layout layout = packedLayout(0, rectangles);
        assertThat(layout.getWidth(), is(1));
        assertThat(layout.getHeight(), is(1));
        assertThat(layout.getRectangles().size(), is(0));
    }

    @Test
    public void testBasic1() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList(rect("0", 0, 16, 16),
                            rect("1", 1, 16, 16),
                            rect("2", 2, 16, 16),
                            rect("3", 3, 16, 16));

        Layout layout = packedLayout(0, rectangles);
        assertThat(layout.getWidth(), is(32));
        assertThat(layout.getHeight(), is(32));
        assertRect(layout, 0, "0", 0, 0, 0);
        assertRect(layout, 1, "1", 1, 0, 16);
        assertRect(layout, 2, "2", 2, 16, 0);
        assertRect(layout, 3, "3", 3, 16, 16);
    }

    @Test
    public void testBasic2() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList(rect("0", 0, 16, 16),
                            rect("1", 1, 8, 8),
                            rect("2", 2, 8, 8),
                            rect("3", 3, 8, 8),
                            rect("4", 4, 8, 8),
                            rect("5", 5, 8, 8),
                            rect("6", 6, 8, 8));

        // 0034
        // 00
        // 15
        // 26
        Layout layout = packedLayout(0, rectangles);
        assertThat(layout.getWidth(), is(32));
        assertThat(layout.getHeight(), is(32));
        assertRect(layout, 0, "0", 0, 0, 0);
        assertRect(layout, 1, "1", 1, 0, 16);
        assertRect(layout, 2, "2", 2, 0, 24);
        assertRect(layout, 3, "3", 3, 16, 0);
        assertRect(layout, 4, "4", 4, 24, 0);
        assertRect(layout, 5, "5", 5, 8, 16);
        assertRect(layout, 6, "6", 6, 8, 24);
    }

    @Test
    public void testBasic3() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList(rect("0", 0, 512, 128));

        Layout layout = packedLayout(0, rectangles);
        assertThat(layout.getWidth(), is(512));
        assertThat(layout.getHeight(), is(128));
        assertRect(layout, 0, "0", 0, 0, 0);
    }

    @Test
    public void testBasic4() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList(rect("0", 0, 32, 12),
                            rect("1", 1, 16, 2),
                            rect("2", 2, 16, 2));

        Layout layout = packedLayout(0, rectangles);
        assertThat(layout.getWidth(), is(32));
        assertThat(layout.getHeight(), is(16));
        assertRect(layout, 0, "0", 0, 0, 0);
        assertRect(layout, 1, "1", 1, 0, 12);
        assertRect(layout, 2, "2", 2, 0, 14);
    }

    @Test
    public void testBasicMargin1() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList(rect("0", 0, 16, 16),
                            rect("1", 1, 16, 16),
                            rect("2", 2, 16, 16),
                            rect("3", 3, 16, 16));

        Layout layout = packedLayout(2, rectangles);
        assertThat(layout.getWidth(), is(64));
        assertThat(layout.getHeight(), is(64));
        assertRect(layout, 0, "0", 0, 0, 0);
        assertRect(layout, 1, "1", 1, 0, (16 + 2));
        assertRect(layout, 2, "2", 2, (16 + 2), 0);
        assertRect(layout, 3, "3", 3, 0, (16 + 2) * 2);
    }

    @Test
    public void testBasicMargin2() {
        List<TextureSetLayout.Rect> rectangles
            = Arrays.asList(rect("0", 0, 15, 15),
                            rect("1", 1, 15, 15),
                            rect("2", 2, 15, 15),
                            rect("3", 3, 15, 15));

        Layout layout = packedLayout(2, rectangles);
        assertThat(layout.getWidth(), is(64));
        assertThat(layout.getHeight(), is(64));
        assertRect(layout, 0, "0", 0, 0, 0);
        assertRect(layout, 1, "1", 1, 0, (15 + 2));
        assertRect(layout, 2, "2", 2, (15 + 2), 0);
        assertRect(layout, 3, "3", 3, 0, (15 + 2) * 2);
    }

    @Test
    public void testThinStrip() {
        List<TextureSetLayout.Rect> rectangles = Arrays.asList(rect("0", 0, 1, 16));

        Layout layout = packedLayout(0, rectangles);
        assertThat(layout.getWidth(), is(1));
        assertThat(layout.getHeight(), is(16));
        assertRect(layout, 0, "0", 0, 0, 0);
    }

    private Rect rect(String id, int index, int w, int h) {
        return new Rect(id, index, w, h);
    }


    private static int nextLength(int a, int b) {
        return a + b;
    }

    private List<Rect> createSampleRectangles(int scale) {
        final int startRange = 2;
        final int endRange = 10;

        List<Rect> rects = new ArrayList<Rect>();

        int previousY = 1;
        for (int y=startRange; y<endRange; ++y) {
            int yLength = nextLength(y, previousY) * scale;
            int previousX = 1;
            for (int x=startRange; x<endRange; ++x) {
                int xLength = nextLength(x, previousX) * scale;
                rects.add(rect(String.format("%d", rects.size()), rects.size(), xLength, yLength));
                previousX = x;
            }
            previousY = y;
        }

        return rects;
    }

    @Test
    public void testAllIncluded() {
        List<Rect> rectangles = createSampleRectangles(1);
        Layout layout = packedLayout(0, rectangles);

        HashSet<String> recordedIds = new HashSet<String>();

        for (Rect r : layout.getRectangles()) {
            assertFalse(recordedIds.contains(r.id));
            recordedIds.add(r.id);
        }

        assertEquals(recordedIds.size(), rectangles.size());
    }

    private static boolean isOverlapping(Rect a, Rect b) {
        if (a.x >= b.x + b.width) {
            return false;
        }
        if (a.y >= b.y + b.height) {
            return false;
        }
        if (a.x + a.width <= b.x) {
            return false;
        }
        if (a.y + a.height <= b.y) {
            return false;
        }
        return true;
    }

    @Test
    public void testNoOverlaps() {
        List<Rect> rectangles = createSampleRectangles(1);
        Layout layout = packedLayout(0, rectangles);
        List<Rect> outputRectangles = layout.getRectangles();
        int numRectangles = outputRectangles.size();

        for (int i=0; i<numRectangles; ++i) {
            for (int j=i+1; j<numRectangles; ++j) {
                assertFalse(isOverlapping(outputRectangles.get(i), outputRectangles.get(j)));
            }
        }
    }

    @Test
    public void testGridLayout1() {

        List<Rect> rectangles
            = Arrays.asList(rect("0", 0, 16, 4),
                            rect("1", 1, 16, 4),
                            rect("2", 2, 16, 4),
                            rect("3", 3, 16, 4));

        Layout layout = gridLayout(0, rectangles, new Grid(2,2) );

        assertEquals( layout.getWidth(), 32 );
        assertEquals( layout.getHeight(), 8 );
    }

    @Test
    public void testGridLayout2() {

        List<Rect> rectangles
            = Arrays.asList(rect("0", 0, 32, 16));

        Layout layout = gridLayout(0, rectangles, new Grid(2,2) );

        assertEquals( layout.getWidth(), 64 );
        assertEquals( layout.getHeight(), 32 );
    }

    @Test
    public void testGridNoOverlaps() {
        List<Rect> rectangles
            = Arrays.asList(rect("0", 0, 16, 4),
                            rect("1", 1, 16, 4),
                            rect("2", 2, 16, 4),
                            rect("3", 3, 16, 4));
        Layout layout = gridLayout(0, rectangles, new Grid(2,2) );
        List<Rect> outputRectangles = layout.getRectangles();
        int numRectangles = outputRectangles.size();

        for (int i=0; i<numRectangles; ++i) {
            for (int j=i+1; j<numRectangles; ++j) {
                assertFalse(isOverlapping(outputRectangles.get(i), outputRectangles.get(j)));
            }
        }
    }

    @Test
    public void testLargeLayout() {
        List<Rect> rectangles
            = Arrays.asList(rect("0", 0, 1000, 800),
                            rect("1", 1, 800, 1000),
                            rect("2", 2, 1000, 100),
                            rect("3", 3, 800, 100));
        Layout layout = packedLayout(0, rectangles);

        assertEquals(layout.getWidth(), 2048);
        assertEquals(layout.getHeight(), 1024);
    }
}
