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

package com.defold.editor;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Hyperlink;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import javafx.scene.text.TextFlow;
import javafx.stage.Stage;
import javafx.stage.StageStyle;

import java.awt.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URI;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Splash {

    private Stage stage;
    private StringProperty launchError = new SimpleStringProperty();
    private BooleanProperty errorShowing = new SimpleBooleanProperty(false);
    private BooleanProperty shown = new SimpleBooleanProperty(false);

    public Splash() {
    }

    private static Hyperlink hyperlink(String text, String url) {
        Hyperlink hyperlink = new Hyperlink(text);
        hyperlink.setOnAction(event -> new Thread(() -> {
            try {
                Desktop.getDesktop().browse(URI.create(url));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }).start());
        return hyperlink;
    }

    private static Text text(String str) {
        Text text = new Text(str);
        text.getStyleClass().add("text");
        return text;
    }

    private static String unescape(String str) {
        return str.replace("\\n", "\n");
    }

    private static List<Node> stringToTextFlowNodes(String str) {
        List<Node> ret = new ArrayList<>();
        int i = 0;
        while (i < str.length()) {
            if (str.charAt(i) == '[') {
                // parse link
                int textStartIndex = i + 1; // skip '['
                int textEndIndex = str.indexOf(']', textStartIndex);
                String text = unescape(str.substring(textStartIndex, textEndIndex));
                int urlStartIndex = textEndIndex + 2; // skip ']' and then '('
                int urlEndIndex = str.indexOf(')', urlStartIndex);
                String url = str.substring(urlStartIndex, urlEndIndex);

                ret.add(hyperlink(text, url));

                i = urlEndIndex + 1;
            } else {
                // parse text
                int nextUrlIndex = str.indexOf('[', i);
                int textEndIndex = nextUrlIndex == -1 ? str.length() : nextUrlIndex;

                ret.add(text(unescape(str.substring(i, textEndIndex))));

                i = textEndIndex;
            }
        }
        return ret;
    }

    private static List<String> readTips() throws IOException {
        InputStream tipsResource = Thread.currentThread()
                .getContextClassLoader()
                .getResourceAsStream("tips.txt");
        List<String> tips = new ArrayList<>();
        InputStreamReader reader = new InputStreamReader(tipsResource, Charset.forName("UTF-8"));
        try (BufferedReader bufferedReader = new BufferedReader(reader)) {
            String line;
            while ((line = bufferedReader.readLine()) != null) {
                if (line.startsWith("#") || line.isEmpty())
                    continue;
                tips.add(line);
            }
        }
        return tips;
    }

    private static String randomElement(List<String> tips) {
        return tips.get(new Random().nextInt(tips.size()));
    }

    public void show() throws IOException {
        Object root = FXMLLoader.load(this.getClass().getResource("/splash.fxml"));
        Scene scene = new Scene((Parent) root);
        scene.setFill(Color.TRANSPARENT);
        stage = new Stage();
        stage.initStyle(StageStyle.TRANSPARENT);
        stage.getIcons().add(new Image(Splash.class.getResourceAsStream("/logo_blue.png")));
        stage.setScene(scene);

        TextFlow startupTipFlow = (TextFlow) scene.lookup("#startup-tip");
        startupTipFlow.visibleProperty().bind(errorShowing.not());
        startupTipFlow.getChildren().setAll(stringToTextFlowNodes(randomElement(readTips())));

        Label launchErrorLabel = (Label) scene.lookup("#launch-error");
        launchErrorLabel.textProperty().bind(launchError);
        launchErrorLabel.visibleProperty().bind(errorShowing);

        Button errorButton = (Button) scene.lookup("#error-button");
        errorButton.visibleProperty().bind(errorShowing);
        errorButton.setOnAction((event) -> System.exit(1));

        // Footer data; version and copyright
        String versionString = System.getProperty("defold.version");
        String channelName = System.getProperty("defold.channel");
        String sha1 = System.getProperty("defold.editor.sha1");

        versionString = (versionString == null || versionString.isEmpty()) ? "No version" : versionString;
        channelName = (channelName == null || channelName.isEmpty()) ? "No channel" : channelName;
        channelName = channelName.equals("editor-alpha") ? "" : "   •   " + channelName;
        sha1 = (sha1 == null || sha1.isEmpty()) ? "no sha1" : sha1;
        if (sha1.length() > 7) {
            sha1 = sha1.substring(0, 7);
        }

        Label channelLabel = (Label) scene.lookup("#version");
        channelLabel.setText(versionString + " (" + sha1 + ")" + channelName);
        channelLabel.setVisible(true);

        stage.setOnShown(event -> shown.set(true));
        stage.show();
    }

    public void close() {
        stage.close();
    }

    public void setLaunchError(String value) {
        launchError.setValue(value);
    }

    public String getLaunchError() {
        return launchError.getValue();
    }

    public StringProperty launchErrorProperty() {
        return launchError;
    }

    public void setErrorShowing(boolean value) {
        errorShowing.setValue(value);
    }

    public boolean getErrorShowing() {
        return errorShowing.getValue();
    }

    public BooleanProperty errorShowingProperty() {
        return errorShowing;
    }

    public BooleanProperty shownProperty() {
        return shown;
    }
}
