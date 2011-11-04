package com.dynamo.cr.goprot;

import javax.inject.Singleton;

import org.eclipse.core.commands.operations.IOperationHistory;
import org.eclipse.core.commands.operations.IUndoContext;
import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResourceChangeEvent;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.IActionBars;
import org.eclipse.ui.IEditorInput;
import org.eclipse.ui.IEditorSite;
import org.eclipse.ui.IFileEditorInput;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.actions.ActionFactory;
import org.eclipse.ui.operations.RedoActionHandler;
import org.eclipse.ui.operations.UndoActionHandler;
import org.eclipse.ui.progress.IProgressService;

import com.dynamo.cr.editor.core.EditorUtil;
import com.dynamo.cr.editor.core.inject.LifecycleModule;
import com.dynamo.cr.goprot.core.ILogger;
import com.dynamo.cr.goprot.core.INodeView;
import com.dynamo.cr.goprot.core.NodeManager;
import com.dynamo.cr.goprot.core.NodeModel;
import com.dynamo.cr.goprot.gameobject.GameObjectNode;
import com.dynamo.cr.goprot.gameobject.GameObjectPresenter;
import com.dynamo.cr.goprot.sprite.SpriteNode;
import com.dynamo.cr.goprot.sprite.SpritePresenter;
import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Injector;

public class Editor extends AbstractDefoldEditor {

    private IContainer contentRoot;
    private LifecycleModule module;
    private NodeManager manager;

    class Module extends AbstractModule {
        @Override
        protected void configure() {
            bind(INodeView.class).to(NodeView.class).in(Singleton.class);
            bind(NodeModel.class).in(Singleton.class);
            bind(NodeManager.class).in(Singleton.class);
            bind(Editor.class).toInstance(Editor.this);

            bind(IOperationHistory.class).toInstance(history);
            bind(IUndoContext.class).toInstance(undoContext);
            bind(UndoActionHandler.class).toInstance(undoHandler);
            bind(RedoActionHandler.class).toInstance(redoHandler);

            bind(ILogger.class).to(Logger.class);

            bind(IContainer.class).toInstance(contentRoot);

        }
    }

    @Override
    public void init(IEditorSite site, IEditorInput input)
            throws PartInitException {

        super.init(site, input);

        IFileEditorInput fileEditorInput = (IFileEditorInput) input;
        final IFile file = fileEditorInput.getFile();
        this.contentRoot = EditorUtil.findContentRoot(file);
        if (this.contentRoot == null) {
            throw new PartInitException(
                    "Unable to locate content root for project");
        }

        this.module = new LifecycleModule(new Module());
        Injector injector = Guice.createInjector(module);

        final String undoId = ActionFactory.UNDO.getId();
        final String redoId = ActionFactory.REDO.getId();

        IActionBars actionBars = site.getActionBars();
        actionBars.setGlobalActionHandler(undoId, undoHandler);
        actionBars.setGlobalActionHandler(redoId, redoHandler);

        this.manager = injector.getInstance(NodeManager.class);
        // TODO: Replace with extension point
        this.manager.registerPresenter(GameObjectNode.class, injector.getInstance(GameObjectPresenter.class));
        this.manager.registerPresenter(SpriteNode.class, injector.getInstance(SpritePresenter.class));

        IProgressService service = PlatformUI.getWorkbench().getProgressService();

        // TODO: Replace with extension point
        String extension = file.getProjectRelativePath().getFileExtension();
        INodeView.Presenter presenter = null;
        String type = null;
        if (extension.equals("go")) {
            presenter = this.manager.getPresenter(GameObjectNode.class);
            type = "Game Object";
        } else if (extension.equals("sprite")) {
            presenter = this.manager.getPresenter(SpriteNode.class);
            type = "Sprite";
        }

        NodeLoaderRunnable loader = new NodeLoaderRunnable(file, presenter, type);
        try {
            service.runInUI(service, loader, null);
            if (loader.exception != null) {
                throw new PartInitException(loader.exception.getMessage(),
                        loader.exception);
            }
        } catch (Throwable e) {
            throw new PartInitException(e.getMessage(), e);
        }
    }

    @Override
    protected void doReload(IFile file) {
        // TODO Auto-generated method stub

    }

    @Override
    protected void handleResourceChanged(IResourceChangeEvent event) {
        // TODO Auto-generated method stub

    }

    @Override
    public void doSave(IProgressMonitor monitor) {
        // TODO Auto-generated method stub

    }

    @Override
    public void doSaveAs() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isDirty() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean isSaveAsAllowed() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void createPartControl(Composite parent) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setFocus() {
        // TODO Auto-generated method stub

    }

}
