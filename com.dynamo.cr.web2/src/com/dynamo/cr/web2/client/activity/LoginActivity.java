package com.dynamo.cr.web2.client.activity;

import com.dynamo.cr.web2.client.ClientFactory;
import com.dynamo.cr.web2.client.place.LoginPlace;
import com.dynamo.cr.web2.client.ui.LoginView;
import com.google.gwt.activity.shared.AbstractActivity;
import com.google.gwt.event.shared.EventBus;
import com.google.gwt.http.client.URL;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.ui.AcceptsOneWidget;

public class LoginActivity extends AbstractActivity implements LoginView.Presenter {
	private ClientFactory clientFactory;

	public LoginActivity(LoginPlace place, ClientFactory clientFactory) {
		this.clientFactory = clientFactory;
	}

	@Override
	public void start(AcceptsOneWidget containerWidget, EventBus eventBus) {
		LoginView loginView = clientFactory.getLoginView();
		containerWidget.setWidget(loginView.asWidget());
		loginView.setPresenter(this);
	}

    @Override
    public void loginGoogle() {
        String url = clientFactory.getDefold().getUrl();

        // The redirectToUrl is the #openid-activity, ie the url redirected to after login
        String redirectToUrl = Window.Location.createUrlBuilder().buildString();
        if (redirectToUrl.lastIndexOf('#') != -1) {
            redirectToUrl = redirectToUrl.substring(0, redirectToUrl.lastIndexOf('#'));
        }
        redirectToUrl += "#openid:{token}_{action}";
        String openAuthUrl = url + "/login/openid/google?redirect_to=" + URL.encodeQueryString(redirectToUrl);
        Window.Location.replace(openAuthUrl);
    }
}
