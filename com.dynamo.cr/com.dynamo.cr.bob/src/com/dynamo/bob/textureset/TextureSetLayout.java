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

package com.dynamo.bob.textureset;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Atlas layout algorithm(s)
 * @author chmu
 *
 */
public class TextureSetLayout {

    public static class Grid {
        public int columns;
        public int rows;

        public Grid( int columns, int rows ) {
            this.columns = columns;
            this.rows    = rows;
        }

        @Override
        public String toString() {
            return String.format("%d columns, %d rows", columns, rows);
        }
    }

    public static class Rect {
        public String id;
        public int index; // for easier keeping the original order
        public int x, y, width, height;
        public boolean rotated;

        public Rect(String id, int index, int x, int y, int width, int height) {
            this.id = id;
            this.index = index;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.rotated = false;
        }

        public Rect(String id, int index, int width, int height) {
            this.id = id;
            this.index = index;
            this.x = -1;
            this.y = -1;
            this.width = width;
            this.height = height;
            this.rotated = false;
        }

        public Rect(Rect other) {
            this.id = other.id;
            this.index = other.index;
            this.x = other.x;
            this.y = other.y;
            this.width = other.width;
            this.height = other.height;
            this.rotated = other.rotated;
        }

        public int area() {
            return width * height;
        }

        @Override
        public String toString() {
            return String.format("[%d, %d, %d, %d]", x, y, width, height);
        }
    }

    public static class Layout {
        private final List<Rect> rectangles;
        private final int width;
        private final int height;
        public Layout(int width, int height, List<Rect> rectangles) {
            this.width = width;
            this.height = height;
            this.rectangles = rectangles;
        }

        public List<Rect> getRectangles() {
            return rectangles;
        }
        public int getWidth() {
            return width;
        }
        public int getHeight() {
            return height;
        }

    }

    public static Layout packedLayout(int margin, List<Rect> rectangles, boolean rotate) {
        if (rectangles.size() == 0) {
            return new Layout(1, 1, new ArrayList<TextureSetLayout.Rect>());
        }

        return createMaxRectsLayout(margin, rectangles, rotate);
    }

    private static int getExponentNextOrMatchingPowerOfTwo(int value) {
        int exponent = 0;
        while (value > (1<<exponent)) {
            ++exponent;
        }
        return exponent;
    }

    public static Layout gridLayout(int margin, List<Rect> rectangles, Grid gridSize ) {

        // We assume here that all images have the same size,
        // since they will be "packed" into a uniformly sized grid.
        int cellWidth  = rectangles.get(0).width;
        int cellHeight = rectangles.get(0).height;

        // Scale layout area so it's a power of two
        int inputWidth  = gridSize.columns * cellWidth + margin*2;
        int inputHeight = gridSize.rows * cellHeight + margin*2;
        int layoutWidth = 1 << getExponentNextOrMatchingPowerOfTwo(inputWidth);
        int layoutHeight = 1 << getExponentNextOrMatchingPowerOfTwo(inputHeight);

        int x = margin;
        int y = margin;

        // Loop through all images and put them right after each other
        Layout layout = new Layout(layoutWidth, layoutHeight, rectangles);
        for (Rect r : layout.getRectangles()) {

            r.x = x;
            r.y = y;

             if (x + cellWidth >= inputWidth - margin) {
                 x = margin;
                 y += cellHeight;
             } else {
                 x += cellWidth;
             }

        }

        return layout;
    }

    /**
     * @param margin
     * @param rectangles
     * @param rotate
     * @return
     */
    public static Layout createMaxRectsLayout(int margin, List<Rect> rectangles, boolean rotate) {
        // Sort by area first, then longest side
        Collections.sort(rectangles, new Comparator<Rect>() {
            @Override
            public int compare(Rect o1, Rect o2) {
                int a1 = o1.width * o1.height;
                int a2 = o2.width * o2.height;
                if (a1 != a2) {
                    return a2 - a1;
                }
                int n1 = o1.width > o1.height ? o1.width : o1.height;
                int n2 = o2.width > o2.height ? o2.width : o2.height;
                return n2 - n1;
            }
        });

        // Calculate total area of rectangles and the max length of a rectangle
        int maxLengthScale = 0;
        int area = 0;
        for (Rect rect : rectangles) {
            area += rect.area();
            maxLengthScale = Math.max(maxLengthScale, rect.width + margin);
            maxLengthScale = Math.max(maxLengthScale, rect.height + margin);
        }

        // Ensure the longest length found in all of the images will fit within one page, irrespective of orientation.
        final int defaultMaxPageSize = 1 << getExponentNextOrMatchingPowerOfTwo(Math.max((int)Math.sqrt(area), maxLengthScale));
        final int defaultMinPageSize = 16;

        MaxRectsLayoutStrategy.Settings settings = new MaxRectsLayoutStrategy.Settings();
        settings.maxPageHeight = defaultMaxPageSize;
        settings.maxPageWidth = defaultMaxPageSize;
        settings.minPageHeight = defaultMinPageSize;
        settings.minPageWidth = defaultMinPageSize;
        settings.paddingX = margin;
        settings.paddingY = margin;
        settings.rotation = rotate;
        settings.square = false;

        MaxRectsLayoutStrategy strategy = new MaxRectsLayoutStrategy(settings);
        List<Layout> layouts = strategy.createLayout(rectangles);

        // Repeat layout creation using alternating increase of width and height until
        // all rectangles can be fit into a single layout
        int iterations = 0;
        while (layouts.size() > 1) {
            iterations++;
            settings.minPageWidth = settings.maxPageWidth;
            settings.minPageHeight = settings.maxPageHeight;
            settings.maxPageHeight *= (iterations % 2) == 0 ? 2 : 1;
            settings.maxPageWidth *= (iterations % 2) == 0 ? 1 : 2;
            layouts = strategy.createLayout(rectangles);
        }

        return layouts.get(0);
    }
}
