package com.dynamo.cr.goprot.test;

import java.io.ByteArrayInputStream;
import java.io.IOException;

import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.runtime.CoreException;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.dynamo.cr.goprot.sprite.AnimationNode;
import com.dynamo.cr.goprot.sprite.SpriteNode;
import com.dynamo.cr.goprot.sprite.SpritePresenter;
import com.google.inject.Module;
import com.google.inject.Singleton;

public class SpriteTest extends AbstractTest {

    class TestModule extends GenericTestModule {
        @Override
        protected void configure() {
            super.configure();
            bind(SpritePresenter.class).in(Singleton.class);
        }
    }

    @Override
    @Before
    public void setup() throws CoreException, IOException {
        super.setup();
        this.model.setRoot(new SpriteNode());
        this.manager.registerPresenter(SpriteNode.class, this.injector.getInstance(SpritePresenter.class));
    }

    // Tests

    @Test
    public void testAddAnimation() throws ExecutionException {
        SpriteNode sprite = (SpriteNode)this.model.getRoot();
        SpritePresenter presenter = (SpritePresenter)this.manager.getPresenter(SpriteNode.class);
        AnimationNode animation = new AnimationNode();
        presenter.onAddAnimation(sprite, animation);
        assertEquals(1, sprite.getChildren().size());
        assertEquals(animation, sprite.getChildren().get(0));
        verifyUpdate(sprite);

        undo();
        assertEquals(0, sprite.getChildren().size());
        verifyUpdate(sprite);

        redo();
        assertEquals(1, sprite.getChildren().size());
        assertEquals(animation, sprite.getChildren().get(0));
        verifyUpdate(sprite);
    }

    @Test
    public void testRemoveAnimation() throws ExecutionException {
        SpriteNode sprite = (SpriteNode)this.model.getRoot();
        SpritePresenter presenter = (SpritePresenter)this.manager.getPresenter(SpriteNode.class);
        AnimationNode animation = new AnimationNode();
        presenter.onAddAnimation(sprite, animation);
        assertEquals(1, sprite.getChildren().size());
        assertEquals(animation, sprite.getChildren().get(0));
        verifyUpdate(sprite);

        undo();
        assertEquals(0, sprite.getChildren().size());
        verifyUpdate(sprite);

        redo();
        assertEquals(1, sprite.getChildren().size());
        assertEquals(animation, sprite.getChildren().get(0));
        verifyUpdate(sprite);
    }

    @Test
    public void testLoading() throws IOException {
        String ddf = "texture: '' width: 1 height: 1";
        SpritePresenter presenter = (SpritePresenter)this.manager.getPresenter(SpriteNode.class);
        presenter.onLoad(new ByteArrayInputStream(ddf.getBytes()));
        assertTrue(this.model.getRoot() instanceof SpriteNode);
    }

    @Override
    Module getModule() {
        return new TestModule();
    }

}
